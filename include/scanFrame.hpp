#pragma once

/**
 * @file scan_frame.hpp
 * @brief Shared data contract between the LED mapper and the LP5899 driver.
 *
 * ScanFrame is the handoff point in the LED pipeline:
 *
 *   LedPanelMapper  (application layer)  -- writes heater status into ScanFrame
 *                        |
 *                    ScanFrame            -- this file
 *                        |
 *   LP5899_PanelDriver  (driver layer)   -- reads ScanFrame and uploads to hardware
 */

#include <Arduino.h>

static constexpr uint8_t PANEL_HEATER_COUNT = 30;
static constexpr uint8_t PANEL_LINE_COUNT   = 8;
static constexpr uint8_t PANEL_COL_COUNT    = 15;
static constexpr uint16_t PANEL_GS_MAX      = 0xFFFF;

/*
 * Scan-frame representation of the real hardware.
 *
 * One LP5890 and reuses R0..R14 / G0..G14 across
 * multiple hardware level line outputs:
 *
 *   line 0: fault  R0-R14, relay   G0-G14
 *   line 1: fault  R15-R29, relay  G15-G29
 *   line 2: localV  G0-G14
 *   line 3: localV  G15-G29
 *   line 4: localI  G0-G14
 *   line 5: localI  G15-G29
 *   line 6: remoteI  G0-G14
 *   line 7: remoteI  G15-G29
 */
struct ScanFrame
{
  uint16_t r[PANEL_LINE_COUNT][PANEL_COL_COUNT] = {};  ///< Red channel (15 brightness values)
  uint16_t g[PANEL_LINE_COUNT][PANEL_COL_COUNT] = {};  ///< Green channel (15 brightness values)
};