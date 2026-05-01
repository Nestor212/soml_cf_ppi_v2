#pragma once
#include <Arduino.h>

/// Total heater channels in the power panel.
/// The power panel controls 30 independent heater modules, each with its own relay,
/// current sensing, voltage monitoring, and temperature measurement.
static constexpr uint8_t HEATER_COUNT = 30;

/// Generic validity flag for indicating whether a measured signal is trustworthy.
/// Used to track the health of all analog and digital measurements throughout the system.
/// INVALID = measurement failed, is stale, or has not been sampled yet
/// VALID   = measurement was successfully acquired within acceptable timeframe
enum class SignalValidity : uint8_t {
  INVALID = 0,
  VALID   = 1
};

/**
 * @brief Published state snapshot for one heater channel.
 * 
 * Contains all measured and commanded information for a single heater module,
 * including I/O measurements, temperature, and their respective validity/timestamp data.
 * This allows consumers to understand both the current state and the freshness of each measurement.
 */
struct HeaterState
{
  /// Timestamp (in milliseconds) when this struct was last updated.
  /// Useful for detecting stale state snapshots or computing measurement age.
  uint32_t stateMs = 0;

  /// Commanded relay output for this heater channel (true = ON, false = OFF).
  /// This is the requested state sent to the hardware relay, not actual hardware feedback.
  /// May differ from actual relay state if communication or hardware failure occurs.
  bool relayCommand = false;

  /// Digital input signal measurements from the heater module
  /// All signals are normalized to boolean (true/false, not raw analog values)
  /// localCurrent  = true when current is detected (active-high logic; high voltage = current flowing)
  /// localVoltage  = true when voltage is NOT detected (active-low logic; ground voltage = no supply)
  /// remoteCurrent = true when remote current sensor is NOT active (active-low logic; for remote monitoring)
  bool localCurrent  = false;
  bool localVoltage  = false;
  bool remoteCurrent = false;

  /// Validity indicator for all I/O measurements (localCurrent, localVoltage, remoteCurrent).
  /// When INVALID, measurements may be stale or uninitialized; ignore their values.
  SignalValidity ioValid = SignalValidity::INVALID;
  
  /// Timestamp (milliseconds) when I/O measurements were last successfully acquired.
  /// Use this to determine measurement age and detect communication timeouts.
  uint32_t ioUpdatedMs   = 0;

  /// Heater module temperature in degrees Celsius (float precision for accuracy).
  /// Measured by the embedded temperature sensor within the heater module itself.
  float moduleTempC = 0.0f;
  
  /// Validity indicator for temperature measurement.
  /// When INVALID, moduleTempC value is stale or uninitialized; do not use for control logic.
  SignalValidity tempValid = SignalValidity::INVALID;
  
  /// Timestamp (milliseconds) when temperature was last successfully measured.
  /// Use this to determine measurement age and detect temperature sensor failures.
  uint32_t tempUpdatedMs   = 0;
};

/**
 * @brief Complete state snapshot for the entire 30-heater power panel.
 * 
 * Aggregates the state of all 30 heater channels plus panel-level health indicators.
 * Provides validity tracking for local I/O measurements and remote monitoring signals,
 * allowing consumers to evaluate data freshness and system health at a glance.
 */
struct PowerPanelState
{
  /// Array of individual heater states (30 channels total).
  /// Each element represents one heater module with its measurements and commands.
  HeaterState heaters[HEATER_COUNT];
  
  /// Timestamp (milliseconds) when this complete panel state snapshot was assembled.
  /// Represents the moment the entire measurement collection was completed.
  uint32_t snapshotMs = 0;

  /// Validity indicator for all locally-measured I/O signals from the power panel itself.
  /// When INVALID, local measurements are unreliable; communication with panel may be broken.
  /// When VALID, all heater I/O inputs (current, voltage) have been successfully read.
  SignalValidity localIoValid  = SignalValidity::INVALID;
  
  /// Validity indicator for remote I/O signals received from external monitoring system.
  /// When INVALID, remote measurements are unreliable; external monitor communication may be broken.
  /// When VALID, remote current sensors and external inputs are functioning.
  SignalValidity remoteIoValid = SignalValidity::INVALID;

  /// Timestamp (milliseconds) when local I/O measurements were last successfully acquired from the panel.
  /// Use this to detect if local measurements are stale due to communication failures.
  uint32_t localIoUpdatedMs  = 0;
  
  /// Timestamp (milliseconds) when remote I/O signals were last successfully received.
  /// Use this to detect if external monitoring is offline or running late.
  uint32_t remoteIoUpdatedMs = 0;
};