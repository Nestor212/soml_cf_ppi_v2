#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "heaterState.hpp"
#include "pinout.hpp"

/**
 * @brief Mapping to a single PCA9698 GPIO bit.
 */
struct PcaPin {
  uint8_t addr;  // I2C address
  uint8_t bank;  // port 0..4
  uint8_t bit;   // bit 0..7
};

/**
 * @brief Controller for the full 30-heater panel.
 *
 * Owns:
 *  - I2C buses
 *  - full panel state
 *  - PCA mappings + refresh logic
 *
 * Heater = lightweight view into this.
 */
class PowerPanel {
public:
  // ---- Init ----
  void begin(TwoWire& i2c0 = Wire, TwoWire& i2c1 = Wire1);

  // ---- Refresh ----
  bool refreshIo();              // all digital I/O (bank-based)
  bool refreshLocalIOBanks();   // local current + voltage
  bool refreshRemoteIOBanks();  // remote current
  bool refreshTemp(uint8_t heaterIndex); // per-channel ADC
  bool refreshNextTemp();       // round-robin temp update

  // ---- Control ----
  bool setRelay(uint8_t heaterIndex, bool on);

  bool setRelaysFromMask(uint32_t relayMask);
  bool setRelayBanks(const uint8_t banks[5]);

  // Debounced / inferred local current
  bool localCurrentPresent(uint8_t heaterIndex) const;

  // ---- State access ----
  PowerPanelState& state() { return _state; }

  HeaterState& heaterState(uint8_t heaterIndex);

  const HeaterState* heaterStates() const;

private:
  /**
   * Tracks local current activity (pulse → presence).
   */
  struct LocalCurrentTrack {
    bool rawNow = false;
    bool present = false;
    uint32_t lastSeenActiveUs = 0;
    uint32_t lastChangeUs = 0;
  };

  // I2C buses
  TwoWire* _i2c0 = nullptr; // local IO
  TwoWire* _i2c1 = nullptr; // relay + remote

  // Full panel state
  PowerPanelState _state{};

  // Local current tracking (debounce/timeout)
  LocalCurrentTrack _localTrack[HEATER_COUNT]{};

  // Temp round-robin index
  uint8_t _nextTempIndex = 0;

  // ---- Mapping tables (heater → PCA pin) ----
  static const PcaPin localI_[HEATER_COUNT];
  static const PcaPin volt_[HEATER_COUNT];
  static const PcaPin remoteI_[HEATER_COUNT];
  static const PcaPin relayCtrl_[HEATER_COUNT];

  // ---- PCA helpers ----
  TwoWire* wireForAddr_(uint8_t addr) const;
  bool readRegs_(uint8_t addr, uint8_t startReg, uint8_t* dst, uint8_t count) const;
  bool writeRegs_(uint8_t addr, uint8_t startReg, const uint8_t* src, uint8_t count) const;

  bool readGpioBit_(uint8_t addr, uint8_t port, uint8_t bit, bool& stateOut) const;
  bool writeGpioBit_(uint8_t addr, uint8_t port, uint8_t bit, bool value) const;
  bool setPinDirection_(uint8_t addr, uint8_t port, uint8_t bit, bool isInput) const;

  // ---- Temp mux helpers ----
  static uint8_t heaterToTempMuxCh_(uint8_t heaterIndex);
  static void tempMuxSelect_(uint8_t ch);
};