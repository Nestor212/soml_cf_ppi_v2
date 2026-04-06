#pragma once
#include <Arduino.h>

// Total heater channels in the panel
static constexpr uint8_t HEATER_COUNT = 30;

// Generic validity flag for sampled signals
enum class SignalValidity : uint8_t {
  INVALID = 0,
  VALID   = 1
};

/**
 * @brief Published state for one heater channel.
 */
struct HeaterState
{
  uint32_t stateMs = 0;   // last state update time

  // Commanded output state (not hardware feedback)
  bool relayCommand = false;

  // Digital measurements
  bool localCurrent  = false;  // active-high, normalized
  bool localVoltage  = false;  // active-low, normalized
  bool remoteCurrent = false;  // active-low, normalized

  SignalValidity ioValid = SignalValidity::INVALID;
  uint32_t ioUpdatedMs   = 0;

  // Temperature
  float moduleTempC = 0.0f;
  SignalValidity tempValid = SignalValidity::INVALID;
  uint32_t tempUpdatedMs   = 0;
};

/**
 * @brief Full state for the 30-heater power panel.
 */
struct PowerPanelState
{
  HeaterState heaters[HEATER_COUNT];
  uint32_t snapshotMs = 0;

  // Panel-level I/O validity / timestamps
  SignalValidity localIoValid  = SignalValidity::INVALID;
  SignalValidity remoteIoValid = SignalValidity::INVALID;

  uint32_t localIoUpdatedMs  = 0;
  uint32_t remoteIoUpdatedMs = 0;
};