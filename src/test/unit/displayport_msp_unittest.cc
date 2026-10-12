/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Screen load budget and string segmentation of the MSP displayport driver

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "drivers/display.h"
    #include "drivers/osd.h"
    #include "io/displayport_msp.h"
    #include "msp/msp_protocol.h"
    #include "msp/msp_serial.h"
    #include "osd/osd.h"
    #include "pg/displayport_profiles.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/vcd.h"

    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(vcdProfile_t, vcdProfile, PG_VCD_CONFIG, 0);
    PG_REGISTER(displayPortProfile_t, displayPortProfileMsp, PG_DISPLAY_PORT_MSP_CONFIG, 0);

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];
}

#include "gtest/gtest.h"

struct packet_t {
    uint8_t subcmd;
    uint8_t row;
    uint8_t col;
    uint8_t attr;
    std::string text;
};

static std::vector<packet_t> packets;

extern "C" {
    int mspSerialPush(serialPortIdentifier_e, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e, mspVersion_e)
    {
        EXPECT_EQ(MSP_DISPLAYPORT, cmd);
        packet_t p = { data[0], 0, 0, 0, "" };
        if (data[0] == MSP_DP_WRITE_STRING) {
            p = { data[0], data[1], data[2], data[3], std::string((const char *)&data[4], datalen - 4) };
        }
        packets.push_back(p);
        return datalen + 6;
    }

    uint32_t mspSerialTxBytesFree(void) { return UINT32_MAX; }
}

static displayPort_t *init(uint16_t budget)
{
    vcdProfileMutable()->video_system = VIDEO_SYSTEM_HD;
    osdConfigMutable()->canvas_cols = OSD_HD_COLS;
    osdConfigMutable()->canvas_rows = OSD_HD_ROWS;
    displayPortProfileMspMutable()->cellBudget = budget;
    for (int i = 0; i < DISPLAYPORT_SEVERITY_COUNT; i++) {
        displayPortProfileMspMutable()->fontSelection[i] = i;
    }
    debugMode = DEBUG_MSP_DISPLAYPORT;

    displayPort_t *dp = displayPortMspInit();
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);
    packets.clear();

    return dp;
}

static std::vector<packet_t> writes(void)
{
    std::vector<packet_t> w;
    for (const auto &p : packets) {
        if (p.subcmd == MSP_DP_WRITE_STRING) {
            w.push_back(p);
        }
    }
    return w;
}

TEST(DisplayPortMspTest, BlankSpacesAreNotSent)
{
    displayPort_t *dp = init(0);

    displayWrite(dp, 10, 2, DISPLAYPORT_SEVERITY_NORMAL, "  12  ");
    auto w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ(2, w[0].row);
    EXPECT_EQ(12, w[0].col);
    EXPECT_EQ("12", w[0].text);

    packets.clear();
    displayWrite(dp, 0, 5, DISPLAYPORT_SEVERITY_NORMAL, "     ");
    EXPECT_TRUE(packets.empty());
}

TEST(DisplayPortMspTest, ShortGapIsMergedLongGapIsSplit)
{
    displayPort_t *dp = init(0);

    // A gap shorter than a write's header is cheaper to send
    displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, "DBG     1     2");
    auto w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ("DBG     1     2", w[0].text);

    packets.clear();
    displayWrite(dp, 0, 1, DISPLAYPORT_SEVERITY_NORMAL, "A          B");
    w = writes();
    ASSERT_EQ(2u, w.size());
    EXPECT_EQ("A", w[0].text);
    EXPECT_EQ("B", w[1].text);
    EXPECT_EQ(11, w[1].col);
}

TEST(DisplayPortMspTest, SpacesEraseWrittenCells)
{
    displayPort_t *dp = init(0);

    displayWrite(dp, 4, 3, DISPLAYPORT_SEVERITY_NORMAL, "12345");
    displayDrawScreen(dp);

    // An update in place without a clear, as the CMS does: the spaces must reach the device
    packets.clear();
    displayWrite(dp, 4, 3, DISPLAYPORT_SEVERITY_NORMAL, "9      ");
    auto w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ("9    ", w[0].text);

    // After a clear the same spaces are blank again
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);
    packets.clear();
    displayWrite(dp, 4, 3, DISPLAYPORT_SEVERITY_NORMAL, "9      ");
    w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ("9", w[0].text);
}

TEST(DisplayPortMspTest, LongStringIsChunkedNotTruncated)
{
    displayPort_t *dp = init(0);

    const std::string s = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789ABCD";
    displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, s.c_str());
    auto w = writes();
    ASSERT_EQ(2u, w.size());
    EXPECT_EQ(s.substr(0, 30), w[0].text);
    EXPECT_EQ(s.substr(30), w[1].text);
    EXPECT_EQ(30, w[1].col);
}

TEST(DisplayPortMspTest, RowsBeyondTheTrackedScreenAreSentAsTheyAre)
{
    displayPort_t *dp = init(20);

    displayWrite(dp, 0, OSD_HD_ROWS + 1, DISPLAYPORT_SEVERITY_NORMAL, " A B ");
    auto w = writes();
    ASSERT_EQ(1u, w.size());
    EXPECT_EQ(" A B ", w[0].text);
}

TEST(DisplayPortMspTest, AttributeMapping)
{
    displayPort_t *dp = init(0);
    displayPortProfileMspMutable()->fontSelection[DISPLAYPORT_SEVERITY_WARNING] = 0;

    displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_WARNING | DISPLAYPORT_BLINK, "W");
    displayWrite(dp, 0, 1, DISPLAYPORT_SEVERITY_CRITICAL, "C");
    auto w = writes();
    ASSERT_EQ(2u, w.size());
    EXPECT_EQ(DISPLAYPORT_MSP_ATTR_BLINK, w[0].attr);
    EXPECT_EQ(3, w[1].attr);
}

// Screen load: one per cell, plus 0.75 for a cell two columns from the one before it and 1.3 for one
// three or more columns away, taking cells in screen order and counting from the end of the row above.
// The test numbers are the debug fields: 0 the load in whole cells, 1 the writes dropped in the frame.
TEST(DisplayPortMspTest, RewritingCellsIsFree)
{
    displayPort_t *dp = init(21);

    for (int i = 0; i < 10; i++) {
        displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, "0123456789ABCDEFGHIJ");
        displayDrawScreen(dp);
    }
    EXPECT_EQ(10u, writes().size());
    EXPECT_EQ(20, debug[0]);
    EXPECT_EQ(0, debug[1]);
}

TEST(DisplayPortMspTest, FrameOverBudgetDropsWrites)
{
    displayPort_t *dp = init(22);

    displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, "123456789012345");  // 15
    displayWrite(dp, 0, 1, DISPLAYPORT_SEVERITY_NORMAL, "1234567890");       // 15 + 10 + 1.3 = 26.3
    displayWrite(dp, 0, 2, DISPLAYPORT_SEVERITY_NORMAL, "12345");            // 15 + 5 + 1.3 = 21.3
    displayDrawScreen(dp);

    auto w = writes();
    ASSERT_EQ(2u, w.size());
    EXPECT_EQ("123456789012345", w[0].text);
    EXPECT_EQ("12345", w[1].text);
    EXPECT_EQ(2, w[1].row);
    EXPECT_EQ(21, debug[0]);
    EXPECT_EQ(1, debug[1]);
}

TEST(DisplayPortMspTest, GapsAreSplitWhenShortOfBudget)
{
    displayPort_t *dp = init(23);

    displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, "123456789012345");
    // On top of 15 and the step down a row (1.3): merged this adds 9, split 3 + 2 * 1.3
    displayWrite(dp, 0, 1, DISPLAYPORT_SEVERITY_NORMAL, "A   B   C");
    auto w = writes();
    ASSERT_EQ(4u, w.size());
    EXPECT_EQ("A", w[1].text);
    EXPECT_EQ("B", w[2].text);
    EXPECT_EQ(4, w[2].col);
    EXPECT_EQ("C", w[3].text);
    EXPECT_EQ(8, w[3].col);
}

TEST(DisplayPortMspTest, NextFrameSplitsGapsAfterOverflow)
{
    displayPort_t *dp = init(28);

    for (int frame = 0; frame < 3; frame++) {
        displayClearScreen(dp, DISPLAY_CLEAR_NONE);
        packets.clear();
        displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, "A   B   C");
        displayWrite(dp, 0, 1, DISPLAYPORT_SEVERITY_NORMAL, "D   E   F");
        displayWrite(dp, 0, 2, DISPLAYPORT_SEVERITY_NORMAL, "G   H   I");
        displayDrawScreen(dp);

        EXPECT_EQ(0, debug[1]);
        if (frame == 0) {
            // Merged the three rows come to 29.6, over budget. The first two are, the last is split: 26.2
            EXPECT_EQ(5u, writes().size());
            EXPECT_EQ(26, debug[0]);
        } else {
            // The last screen needed more with gaps sent, so every gap is split from the start: 19.4
            EXPECT_EQ(9u, writes().size());
            EXPECT_EQ(19, debug[0]);
        }
    }
}

TEST(DisplayPortMspTest, OverflowOfContentBuiltUpWithoutAClearStartsOver)
{
    displayPort_t *dp = init(20);

    displayWrite(dp, 0, 0, DISPLAYPORT_SEVERITY_NORMAL, "123456789012345");
    displayDrawScreen(dp);
    dp->cleared = false;

    // No clear since the last draw, as when the CMS updates in place: clear, and have it repaint
    packets.clear();
    displayWrite(dp, 0, 5, DISPLAYPORT_SEVERITY_NORMAL, "1234567890");
    ASSERT_EQ(2u, packets.size());
    EXPECT_EQ(MSP_DP_CLEAR_SCREEN, packets[0].subcmd);
    EXPECT_EQ("1234567890", packets[1].text);
    EXPECT_TRUE(dp->cleared);
    displayDrawScreen(dp);

    // Once only, until the owner clears: a page that does not fit cannot make a loop
    packets.clear();
    displayWrite(dp, 0, 6, DISPLAYPORT_SEVERITY_NORMAL, "1234567890ABC");
    EXPECT_TRUE(packets.empty());
    EXPECT_EQ(0, debug[1]);
    displayDrawScreen(dp);
    EXPECT_EQ(1, debug[1]);
}

// Writes the cells one at a time, draws once and returns how many writes were dropped
static int dropped(displayPort_t *dp, int count, int perRow, int col0, int step)
{
    displayClearScreen(dp, DISPLAY_CLEAR_NONE);
    for (int i = 0; i < count; i++) {
        displayWriteChar(dp, col0 + (i % perRow) * step, i / perRow, DISPLAYPORT_SEVERITY_NORMAL, 'X');
    }
    displayDrawScreen(dp);
    return debug[1];
}

// Bench limits on DJI O3 + Goggles 2: 637 cells in solid rows of 53, 620 in solid rows of 50 from
// column 2, 361 cells two columns apart. The model puts each at 637 to 654 cells of load.
TEST(DisplayPortMspTest, MeasuredLimits)
{
    displayPort_t *dp = init(637);
    EXPECT_EQ(0, dropped(dp, 637, 53, 0, 1));
    EXPECT_EQ(637, debug[0]);
    EXPECT_EQ(1, dropped(dp, 638, 53, 0, 1));

    dp = init(637);
    EXPECT_EQ(0, dropped(dp, 620, 50, 2, 1));
    EXPECT_EQ(636, debug[0]);
    EXPECT_EQ(1, dropped(dp, 621, 50, 2, 1));

    // Scattered cells cost more, so fewer of them fit
    dp = init(620);
    EXPECT_EQ(0, dropped(dp, 300, 26, 0, 2));
    EXPECT_LT(0, dropped(dp, 361, 26, 0, 2));
}

TEST(DisplayPortMspTest, NoBudgetOnAnotherCanvas)
{
    // HDZero announces 50x18, which sets the canvas of the OSD
    displayPort_t *dp = init(20);
    osdConfigMutable()->canvas_cols = 50;
    osdConfigMutable()->canvas_rows = 18;

    for (int row = 0; row < 3; row++) {
        displayWrite(dp, 0, row, DISPLAYPORT_SEVERITY_NORMAL, "12345678901234567890");
    }
    EXPECT_EQ(3u, writes().size());
}
