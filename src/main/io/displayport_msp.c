/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_MSP_DISPLAYPORT

#include "build/debug.h"

#include "cli/cli.h"

#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/osd.h"

#include "io/displayport_msp.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

#include "osd/osd.h"

#include "pg/vcd.h"

// MSP_DP_WRITE_STRING carries at most this many characters. DJI O3/Goggles 2 accept 32,
// other receivers may be sized for 30, so a longer string is sent in several writes.
#define MSP_OSD_MAX_STRING_LENGTH 30

// Bytes it costs to start another write: the MSP v1 frame (6) and the write header (4)
#define MSP_DP_WRITE_OVERHEAD 10

// DJI O3 Air Unit + Goggles 2 discard a whole DRAW_SCREEN whose load is over a fixed limit and keep
// showing the previous frame. Only CLEAR_SCREEN frees load. The load of a screen is worked out from
// the cells written since the last clear, spaces included, in screen order: one cell each, plus a
// surcharge when a cell does not touch the one before it, bigger for three or more columns than for
// two. Rewriting a cell is free, erasing it with a space gives nothing back.
//
// The goggles' internals are unknown, so this is a fit to bench measurements. Solid rows of 53 cells
// fit 637 cells, the cheapest case. Rows of 40 to 52 cells, single cells two columns apart, runs of 2
// to 25 cells and pairs all fit at 610 to 655 cells of load, a frame the goggles discarded never comes
// out below 637, and a frame of real OSD elements that fit up to 632. No glyph costs more than a cell.
// Loads are in thousandths of a cell.
#define MSP_DP_CELL   1000
#define MSP_DP_GAP2   750  // The cell is two columns after the one before it
#define MSP_DP_GAP    1300 // Three columns or more, also across rows

// The load is tracked for the default HD canvas only: DJI neither announces a canvas nor uses
// another one, HDZero announces 50x18 (MSP_SET_OSD_CANVAS sets osd_canvas_width/height)
#define MSP_DP_ROWS OSD_HD_ROWS

static displayPort_t mspDisplayPort;
static serialPortIdentifier_e displayPortSerial;

static struct {
    uint64_t written[MSP_DP_ROWS];  // Cells written since the last CLEAR_SCREEN, a bit per cell
    uint32_t demand;                // Load estimate of the screen if no write had been split or dropped
    uint16_t dropped;               // Writes dropped for lack of budget since the last DRAW_SCREEN
    bool splitGaps;                 // The last screen did not fit with gaps merged, so split at every gap
    bool cleared;                   // The screen started with a CLEAR_SCREEN
    bool autoCleared;               // The driver cleared the screen since the owner last did
} mspCells;

typedef struct displayPortMspCommand_s {
    uint8_t command;
    uint8_t row;
    uint8_t col;
    uint8_t attribute;
    uint8_t data[OSD_CHAR_BYTES];
} __attribute__((packed)) displayPortMspCommand_t;

static int output(displayPort_t *displayPort, uint8_t cmd, uint8_t *buf, int len)
{
    UNUSED(displayPort);

    return mspSerialPush(displayPortSerial, cmd, buf, len, MSP_DIRECTION_REPLY, MSP_V1);
}

static int heartbeat(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_HEARTBEAT };

    // heartbeat is used to:
    // a) ensure display is not released by MW OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
    output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));

    return 0;
}

static int grab(displayPort_t *displayPort)
{
    return heartbeat(displayPort);
}

static int release(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_RELEASE };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

// Load budget of the screen in thousandths of a cell, 0 when there is none
static uint32_t loadBudget(void)
{
    const bool defaultCanvas = vcdProfile()->video_system == VIDEO_SYSTEM_HD
        && osdConfig()->canvas_cols == OSD_HD_COLS && osdConfig()->canvas_rows == OSD_HD_ROWS;

    return defaultCanvas ? displayPortProfileMsp()->cellBudget * MSP_DP_CELL : 0;
}

// Load of the cells of one row, without the step in from the row above
static uint32_t rowLoad(uint64_t bits)
{
    const uint64_t starts = bits & ~(bits << 1);                // first cell of each run
    const int gap2 = popcount64(starts & (bits << 2));          // run starting two columns after a cell
    const int gaps = bits ? popcount64(starts) - 1 : 0;          // steps between the runs of the row

    return popcount64(bits) * MSP_DP_CELL + gap2 * MSP_DP_GAP2 + (gaps - gap2) * MSP_DP_GAP;
}

// Load of the screen, with bits instead of what was written in row (or nothing for row -1)
static uint32_t screenLoad(int row, uint64_t bits)
{
    uint32_t load = 0;
    int last = -1;                                              // screen position of the last cell so far

    for (int r = 0; r < MSP_DP_ROWS; r++) {
        const uint64_t b = (r == row) ? bits : mspCells.written[r];

        if (b) {
            const int step = r * OSD_HD_COLS + __builtin_ctzll(b) - last;

            load += rowLoad(b) + (step == 2 ? MSP_DP_GAP2 : (step > 2 ? MSP_DP_GAP : 0));
            last = r * OSD_HD_COLS + 63 - __builtin_clzll(b);
        }
    }

    return load;
}

static int sendClear(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_CLEAR_SCREEN };

    // The OSD repaints much the same screen after every clear
    mspCells.splitGaps = mspCells.demand > loadBudget();
    mspCells.demand = 0;
    mspCells.cleared = true;
    memset(mspCells.written, 0, sizeof(mspCells.written));

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int clearScreen(displayPort_t *displayPort, displayClearOption_e options)
{
    UNUSED(options);

    mspCells.autoCleared = false;

    return sendClear(displayPort);
}

static bool drawScreen(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { MSP_DP_DRAW_SCREEN };
    output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));

    DEBUG_SET(DEBUG_MSP_DISPLAYPORT, 0, screenLoad(-1, 0) / MSP_DP_CELL);  //!< Screen Load
    DEBUG_SET(DEBUG_MSP_DISPLAYPORT, 1, mspCells.dropped);  //!< Writes Dropped
    mspCells.dropped = 0;
    mspCells.cleared = false;

    return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

// Send string as the writes that are needed, a space on a cell that is still blank since the clear
// changes nothing and is left out. Bits for the cells that are sent are added to *cells, and the
// bytes sent are returned, or 0 if emit is not set. A gap of blank cells shorter than mergeGap
// is sent as spaces, as that is cheaper in bytes than starting another write.
static int segments(displayPort_t *displayPort, uint64_t written, uint64_t *cells, uint8_t row, uint8_t col, uint8_t mspAttr, const char *string, int len, int mergeGap, bool emit)
{
    #define BLANK(i) (string[i] == ' ' && col + (i) < 64 && !((written >> (col + (i))) & 1))
    int bytes = 0;

    for (int i = 0; i < len; i++) {
        if (BLANK(i)) {
            continue;
        }

        int end = i + 1;                                        // end of the write that starts at i
        for (int j = end; j < len && j - i < MSP_OSD_MAX_STRING_LENGTH; j++) {
            if (!BLANK(j)) {
                end = j + 1;
            } else if (j - end >= mergeGap - 1) {
                break;
            }
        }

        if (col + i < 64) {
            *cells |= (((uint64_t)1 << (end - i)) - 1) << (col + i);
        }
        if (emit) {
            uint8_t buf[MSP_OSD_MAX_STRING_LENGTH + 4] = { MSP_DP_WRITE_STRING, row, col + i, mspAttr };

            memcpy(&buf[4], &string[i], end - i);
            bytes += output(displayPort, MSP_DISPLAYPORT, buf, end - i + 4);
        }
        i = end - 1;
    }

    return bytes;
    #undef BLANK
}

static int writeString(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, const char *string)
{
    const int len = strlen(string);
    uint8_t mspAttr = displayPortProfileMsp()->fontSelection[attr & (DISPLAYPORT_SEVERITY_COUNT - 1)] & DISPLAYPORT_MSP_ATTR_FONT;

    if (attr & DISPLAYPORT_BLINK) {
        mspAttr |= DISPLAYPORT_MSP_ATTR_BLINK;
    }

    // Beyond the tracked cells every character is sent
    const bool tracked = row < MSP_DP_ROWS && col + len <= 64;
    const uint64_t written = tracked ? mspCells.written[row] : ~(uint64_t)0;
    const uint32_t budget = tracked ? loadBudget() : 0;
    int mergeGap = MSP_DP_WRITE_OVERHEAD;
    uint64_t cells = written;

    if (budget) {
        uint64_t unsplit = 0;

        segments(displayPort, 0, &unsplit, row, col, 0, string, len, MSP_DP_WRITE_OVERHEAD, false);
        if (unsplit) {
            mspCells.demand += rowLoad(unsplit) + MSP_DP_GAP;
        }

        // Spend bytes to keep blank cells blank when the screen is short of budget
        mergeGap = -1;
        for (int gap = mspCells.splitGaps ? 0 : MSP_DP_WRITE_OVERHEAD; gap >= 0 && mergeGap < 0; gap -= MSP_DP_WRITE_OVERHEAD) {
            uint64_t trial = written;

            segments(displayPort, written, &trial, row, col, 0, string, len, gap, false);
            if (screenLoad(row, trial) <= budget) {
                mergeGap = gap;
            }
        }
        if (mergeGap < 0 && !mspCells.cleared && !mspCells.autoCleared) {
            // Content built up over several draws, as the CMS does: start over, split every gap
            // and have the owner repaint the screen. Once, so a page too big cannot loop.
            sendClear(displayPort);
            mspCells.autoCleared = mspCells.splitGaps = true;
            displayPort->cleared = true;
            displayPort->cursorRow = -1;
            return writeString(displayPort, col, row, attr, string);
        }
        if (mergeGap < 0) {
            // Sending it would make the goggles discard the whole screen
            mspCells.dropped++;
            return 0;
        }
    }

    const int bytes = segments(displayPort, written, &cells, row, col, mspAttr, string, len, mergeGap, true);

    if (tracked) {
        mspCells.written[row] = cells;
    }

    return bytes;
}

static int writeSys(displayPort_t *displayPort, uint8_t col, uint8_t row, displayPortSystemElement_e systemElement)
{
    uint8_t syscmd[4];

    syscmd[0] = MSP_DP_SYS;
    syscmd[1] = row;
    syscmd[2] = col;
    syscmd[3] = systemElement;

    return output(displayPort, MSP_DISPLAYPORT, syscmd, sizeof(syscmd));
}

static int writeChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t attr, uint8_t c)
{
    char buf[2];

    buf[0] = c;
    buf[1] = 0;
    return writeString(displayPort, col, row, attr, buf);
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static void redraw(displayPort_t *displayPort)
{
    drawScreen(displayPort);
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree();
}

#ifdef USE_MSP_DISPLAYPORT_FONT
static bool writeFontCharacter(displayPort_t *displayPort, uint16_t addr, const osdCharacter_t *chr)
{
    displayPortMspCommand_t displayPortCommand;

    if (!chr) {
        return false;
    }

    displayPortCommand.command = MSP_DP_FONTCHAR_WRITE;
    displayPortCommand.row = addr & 0xff;
    displayPortCommand.col = (addr >> 8) & 0xff;
    displayPortCommand.attribute = 0;

    memcpy(displayPortCommand.data, chr->data, OSD_CHAR_BYTES);

    int res = output(displayPort, MSP_DISPLAYPORT, (uint8_t*)&displayPortCommand, OSD_CHAR_BYTES + 4);

    // 80ms delay needed to ensure the MSP display has enough time to process the font data
    delay(80);

    return res > 0;
}

static bool checkReady(displayPort_t *displayPort, bool rescan)
{
    UNUSED(rescan);
    if (
        !displayPort ||
        displayPort->deviceType != DISPLAYPORT_DEVICE_TYPE_MSP
    ) {
        return false;
    }

    return true;
}
#endif

static const displayPortVTable_t mspDisplayPortVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeSys = writeSys,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .redraw = redraw,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
#ifdef USE_MSP_DISPLAYPORT_FONT
    .writeFontCharacter = writeFontCharacter,
    .checkReady = checkReady,
#endif
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

displayPort_t *displayPortMspInit(void)
{
    displayInit(&mspDisplayPort, &mspDisplayPortVTable, DISPLAYPORT_DEVICE_TYPE_MSP);

    if (displayPortProfileMsp()->useDeviceBlink) {
        mspDisplayPort.useDeviceBlink = true;
    }

#ifndef USE_OSD_SD
    if (vcdProfile()->video_system != VIDEO_SYSTEM_HD) {
        vcdProfileMutable()->video_system = VIDEO_SYSTEM_HD;
    }
#endif
#ifndef USE_OSD_HD
    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        vcdProfileMutable()->video_system = VIDEO_SYSTEM_AUTO;
    }
#endif

    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD) {
        mspDisplayPort.rows = osdConfig()->canvas_rows;
        mspDisplayPort.cols = osdConfig()->canvas_cols;
    } else {
        const uint8_t displayRows = (vcdProfile()->video_system == VIDEO_SYSTEM_PAL) ? VIDEO_LINES_PAL : VIDEO_LINES_NTSC;
        mspDisplayPort.rows = displayRows + displayPortProfileMsp()->rowAdjust;
        mspDisplayPort.cols = OSD_SD_COLS + displayPortProfileMsp()->colAdjust;
    }

    redraw(&mspDisplayPort);

    return &mspDisplayPort;
}

void displayPortMspSetSerial(serialPortIdentifier_e serialPort)
{
    displayPortSerial = serialPort;
}

serialPortIdentifier_e displayPortMspGetSerial(void) {
    return displayPortSerial;
}
#endif // USE_MSP_DISPLAYPORT
