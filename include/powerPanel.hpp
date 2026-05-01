#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "heaterState.hpp"
#include "ppi_global.hpp"

/// PCA9698 GPIO bit reference (I2C address, port, bit).
struct PcaPin {
  uint8_t addr;  ///< I2C address
  uint8_t bank;  ///< Port 0..4
  uint8_t bit;   ///< Bit 0..7
};

static constexpr uint16_t POWER_PANEL_MAX_SCHEDULE_ELEMENTS = 240;   ///< Max schedule size
static constexpr uint32_t POWER_PANEL_DEFAULT_CYCLE_PERIOD_US = 250000;  ///< Default period

/// Scheduler state: IDLE, ACTIVE, or IO_INVALID.
enum class PowerPanelSchedulerStatus : uint8_t {
  IDLE = 0,
  ACTIVE = 1,
  IO_INVALID = 2
};

/// Snapshot of scheduler state (active/next schedules, timings, status).
struct PowerPanelScheduleSnapshot {
  uint32_t commandCounter = 0;                                  ///< Command sequence number
  int32_t commandIndex = -1;                                    ///< Current command index
  uint64_t commandSentNtpMs = 0;                                ///< Command send time (NTP ms)
  uint32_t commandSentMicros = 0;                               ///< Command send time (µs)
  uint64_t activeStartNtpMs = 0;                                ///< Active schedule start (NTP ms)
  uint64_t nextStartNtpMs = 0;                                  ///< Next schedule start (NTP ms)
  uint32_t cyclePeriodUs = POWER_PANEL_DEFAULT_CYCLE_PERIOD_US; ///< Cycle period (µs)
  int32_t cycleOffsetUs = 0;                                    ///< Cycle offset (µs)
  uint16_t activeScheduleSize = 0;                              ///< Active schedule count
  uint16_t nextScheduleSize = 0;                                ///< Next schedule count
  uint32_t activeSchedule[POWER_PANEL_MAX_SCHEDULE_ELEMENTS] = {0};  ///< Active command list
  uint32_t nextSchedule[POWER_PANEL_MAX_SCHEDULE_ELEMENTS] = {0};    ///< Pending command list
  PowerPanelSchedulerStatus status = PowerPanelSchedulerStatus::IDLE; ///< Scheduler state
};

/// Master 30-heater panel controller. Owns I2C buses, state, PCA mappings.
class PowerPanel {
public:
  /// Initialize I2C buses.
  void begin(TwoWire& i2c0 = Wire, TwoWire& i2c1 = Wire1);

  /// Read all digital I/O measurements (bank-based).
  bool refreshIo();
  /// Read local current + voltage.
  bool refreshLocalIOBanks();
  /// Read remote current.
  bool refreshRemoteIOBanks();
  /// Read per-channel temperature (ADC).
  bool refreshTemp(uint8_t heaterIndex);
  /// Update next temperature channel (round-robin).
  bool refreshNextTemp();

  /// Set relay state for single heater.
  bool setRelay(uint8_t heaterIndex, bool on);

  /// Set relays from 30-bit mask.
  bool setRelaysFromMask(uint32_t relayMask);
  /// Set relay banks (5 ports, 6 bits ea).
  bool setRelayBanks(const uint8_t banks[5]);

  /// Load next schedule list (activate on demand or at start time).
  bool writeNextList(const uint32_t* list, uint16_t size, bool applyStart, uint64_t startNtpMs);
  /// Update scheduler state based on current time.
  bool schedulerTick(uint64_t nowNtpMs);
  /// Capture current scheduler snapshot.
  void getScheduleSnapshot(PowerPanelScheduleSnapshot& out) const;

  /// Check debounced local current (pulse → presence).
  bool localCurrentPresent(uint8_t heaterIndex) const;

  /// Access full panel state.
  /// Access full panel state.
  PowerPanelState& state() { return _state; }

  /// Access single heater state.
  HeaterState& heaterState(uint8_t heaterIndex);

  /// Access heater state array.
  const HeaterState* heaterStates() const;

private:
  /// Local current activity tracker (debounce/timeout).
  struct LocalCurrentTrack {
    bool rawNow = false;
    bool present = false;
    uint32_t lastSeenActiveUs = 0;
    uint32_t lastChangeUs = 0;
  };

  // I2C buses
  TwoWire* _i2c0 = nullptr;                         ///< I2C0 (local IO)
  TwoWire* _i2c1 = nullptr;                         ///< I2C1 (relay + remote)

  // Full panel state
  PowerPanelState _state{};                         ///< Full panel state

  // Local current tracking (debounce/timeout)
  LocalCurrentTrack _localTrack[HEATER_COUNT]{};   ///< Local current debouncing

  // Temp round-robin index
  uint8_t _nextTempIndex = 0;                       ///< Round-robin temp index

  // Scheduler state
  PowerPanelScheduleSnapshot _sched{};              ///< Scheduler snapshot
  uint16_t _activeIndex = 0;                        ///< Current schedule index
  bool _nextStartValid = false;                     ///< Next start time valid
  bool _activeStartValid = false;                   ///< Active start time valid
  uint64_t _idealCycleNtpMs = 0;                    ///< Ideal cycle start (NTP ms)
  uint32_t _nextCycleUs = 0;                        ///< Next cycle duration (µs)
  bool _schedulerInitialized = false;               ///< Scheduler init flag

  // ---- Mapping tables (heater → PCA pin) ----
  /// Heater → PCA pin mapping tables
  static const PcaPin localI_[HEATER_COUNT];        ///< Local current GPIO pins
  static const PcaPin volt_[HEATER_COUNT];          ///< Voltage GPIO pins
  static const PcaPin remoteI_[HEATER_COUNT];       ///< Remote current GPIO pins
  static const PcaPin relayCtrl_[HEATER_COUNT];     ///< Relay control GPIO pins

  // ---- PCA helpers ----
  /// PCA I2C register operations
  TwoWire* wireForAddr_(uint8_t addr) const;
  bool readRegs_(uint8_t addr, uint8_t startReg, uint8_t* dst, uint8_t byte_count) const;
  bool writeRegs_(uint8_t addr, uint8_t startReg, const uint8_t* src, uint8_t byte_count) const;

  bool readGpioBit_(uint8_t addr, uint8_t port, uint8_t bit, bool& stateOut) const;  ///< Read single GPIO bit
  bool writeGpioBit_(uint8_t addr, uint8_t port, uint8_t bit, bool value) const;     ///< Write single GPIO bit
  bool setPinDirection_(uint8_t addr, uint8_t port, uint8_t bit, bool isInput) const;  ///< Configure GPIO direction

  /// ADC mux and scheduler helpers
  static uint8_t heaterToTempMuxCh_(uint8_t heaterIndex);  ///< ADC mux channel conversion
  static void tempMuxSelect_(uint8_t ch);                  ///< Set ADC mux channel
  bool promoteNextSchedule_(uint64_t nowNtpMs);            ///< Activate next schedule at time
  PowerPanelSchedulerStatus schedulerStatus_() const;      ///< Compute scheduler status
};