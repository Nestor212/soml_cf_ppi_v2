#pragma once

#include <Arduino.h>

static constexpr uint8_t PANEL_HEATER_COUNT = 30;
static constexpr uint8_t PANEL_LINE_COUNT   = 8;
static constexpr uint8_t PANEL_COL_COUNT    = 15;
static constexpr uint16_t PANEL_GS_MAX      = 0xFFFF;

/*
 * Scan-frame representation of the real hardware.
 *
 * One LP5890 and reuses R0..R14 / G0..G14 across
 * multiple line outputs:
 *
 *   line 0: fault  0..14 on R, relay   0..14 on G
 *   line 1: fault 15..29 on R, relay  15..29 on G
 *   line 2: localV  0..14 on G
 *   line 3: localV 15..29 on G
 *   line 4: localI  0..14 on G
 *   line 5: localI 15..29 on G
 *   line 6: remoteI  0..14 on G
 *   line 7: remoteI 15..29 on G
 */
struct ScanFrame
{
  uint16_t r[PANEL_LINE_COUNT][PANEL_COL_COUNT] = {};  ///< Red channel (15 brightness values)
  uint16_t g[PANEL_LINE_COUNT][PANEL_COL_COUNT] = {};  ///< Green channel (15 brightness values)
};