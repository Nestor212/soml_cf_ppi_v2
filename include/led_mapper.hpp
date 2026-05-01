#pragma once

#include <Arduino.h>
#include "scanFrame.hpp"
#include "heaterState.hpp"

/**
 * @brief Maps heater states to LED panel visualization.
 * Translates relay status, measurements, and temperature into LED intensities.
 * Supports incremental single-heater or full-panel rendering.
 */
class LedPanelMapper
{
public:
  /// RED: fault/error condition; GREEN: healthy/valid; OFF: invalid/inactive
  static constexpr uint16_t RED_ON   = PANEL_GS_MAX;
  static constexpr uint16_t GREEN_ON = PANEL_GS_MAX;
  static constexpr uint16_t OFF      = 0;

  /// Clears all LED intensities to OFF.
  void clear(ScanFrame& frame);
  
  /// Render LED display for a single heater channel.
  /// @param heaterIndex  Channel index (0-29)
  /// @param st           Heater state to render
  /// @param frame        Scan frame to update
  void renderOne(uint8_t heaterIndex, const HeaterState& st, ScanFrame& frame);
  
  /// Render LED display for all heater channels.
  /// @param states  Array of HeaterState structures
  /// @param count   Number of states (typically HEATER_COUNT = 30)
  /// @param frame   Scan frame to populate
  void renderAll(const HeaterState* states, uint8_t count, ScanFrame& frame);

private:
  /// LED panel column index for heater channel.
  static uint8_t colFor_(uint8_t heaterIndex);
  /// LED panel bank/row index for heater channel.
  static uint8_t bankFor_(uint8_t heaterIndex);

  /// Fault status LED line (RED=fault, OFF=healthy).
  static uint8_t faultLineFor_(uint8_t heaterIndex);
  /// Relay state LED line (GREEN=ON, OFF=OFF).
  static uint8_t relayLineFor_(uint8_t heaterIndex);
  /// Local voltage LED line (GREEN=detected, OFF=absent).
  static uint8_t localVLineFor_(uint8_t heaterIndex);
  /// Local current LED line (GREEN=detected, OFF=absent).
  static uint8_t localILineFor_(uint8_t heaterIndex);
  /// Remote current LED line (GREEN=detected, OFF=absent).
  static uint8_t remoteILineFor_(uint8_t heaterIndex);

  /// Detect fault condition (any INVALID measurement).
  /// @param st Heater state to check
  /// @return true if fault detected
  bool computeFault_(const HeaterState& st) const;
};