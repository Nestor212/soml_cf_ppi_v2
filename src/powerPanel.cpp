#include "powerPanel.hpp"

/**
 * ============================================================
 * PCA9698 register map + constants
 * ============================================================
 *
 * These are fixed addresses used across all PCA devices.
 */
namespace
{
  constexpr uint8_t PCA_REG_IP0   = 0x00;  // Input port base
  constexpr uint8_t PCA_REG_OP0   = 0x08;  // Output port base
  constexpr uint8_t PCA_REG_IOC0  = 0x18;  // Direction control
  constexpr uint8_t PCA_REG_MSK0  = 0x20;  // Interrupt mask
  constexpr uint8_t PCA_REG_MODE_DEFAULT  = 0x2A;  // Mode register Default
  constexpr uint8_t PCA_REG_MODE_OUTPUTS_UPDATE_ON_STOP = 0x00;

  // Default mode ensures predictable startup behavior
  constexpr uint8_t PCA_MODE_DEFAULT = 0x02;

  // Time window for considering local current "present"
  // even if signal drops briefly. Based on 60Hz AC, we expect positive half-wave every ~8.3ms, 
  // so we check more frequently than that and use a timeout to infer presence.
  constexpr uint32_t LOCAL_PRESENT_TIMEOUT_US = 10000;
}

/**
 * ============================================================
 * Static mapping tables
 * ============================================================
 *
 * These define how logical heater indices map to physical
 * PCA pins (addr + bank + bit).
 */

const PcaPin PowerPanel::localI_[HEATER_COUNT] =
{
  {PCA_ADDR_LOCAL_IO_0,0,0}, {PCA_ADDR_LOCAL_IO_0,0,2}, {PCA_ADDR_LOCAL_IO_0,0,4}, {PCA_ADDR_LOCAL_IO_0,0,6}, {PCA_ADDR_LOCAL_IO_0,1,0},
  {PCA_ADDR_LOCAL_IO_0,1,2}, {PCA_ADDR_LOCAL_IO_0,1,4}, {PCA_ADDR_LOCAL_IO_0,1,6}, {PCA_ADDR_LOCAL_IO_0,4,7}, {PCA_ADDR_LOCAL_IO_0,4,5},
  {PCA_ADDR_LOCAL_IO_0,4,3}, {PCA_ADDR_LOCAL_IO_0,4,1}, {PCA_ADDR_LOCAL_IO_0,3,7}, {PCA_ADDR_LOCAL_IO_0,3,5}, {PCA_ADDR_LOCAL_IO_0,3,3},
  {PCA_ADDR_LOCAL_IO_1,0,0}, {PCA_ADDR_LOCAL_IO_1,0,2}, {PCA_ADDR_LOCAL_IO_1,0,4}, {PCA_ADDR_LOCAL_IO_1,0,6}, {PCA_ADDR_LOCAL_IO_1,1,0},
  {PCA_ADDR_LOCAL_IO_1,1,2}, {PCA_ADDR_LOCAL_IO_1,1,4}, {PCA_ADDR_LOCAL_IO_1,1,6}, {PCA_ADDR_LOCAL_IO_1,4,7}, {PCA_ADDR_LOCAL_IO_1,4,5},
  {PCA_ADDR_LOCAL_IO_1,4,3}, {PCA_ADDR_LOCAL_IO_1,4,1}, {PCA_ADDR_LOCAL_IO_1,3,7}, {PCA_ADDR_LOCAL_IO_1,3,5}, {PCA_ADDR_LOCAL_IO_1,3,3}
};

const PcaPin PowerPanel::volt_[HEATER_COUNT] =
{
  {PCA_ADDR_LOCAL_IO_0,0,1}, {PCA_ADDR_LOCAL_IO_0,0,3}, {PCA_ADDR_LOCAL_IO_0,0,5}, {PCA_ADDR_LOCAL_IO_0,0,7}, {PCA_ADDR_LOCAL_IO_0,1,1},
  {PCA_ADDR_LOCAL_IO_0,1,3}, {PCA_ADDR_LOCAL_IO_0,1,5}, {PCA_ADDR_LOCAL_IO_0,1,7}, {PCA_ADDR_LOCAL_IO_0,4,6}, {PCA_ADDR_LOCAL_IO_0,4,4},
  {PCA_ADDR_LOCAL_IO_0,4,2}, {PCA_ADDR_LOCAL_IO_0,4,0}, {PCA_ADDR_LOCAL_IO_0,3,6}, {PCA_ADDR_LOCAL_IO_0,3,4}, {PCA_ADDR_LOCAL_IO_0,3,2},
  {PCA_ADDR_LOCAL_IO_1,0,1}, {PCA_ADDR_LOCAL_IO_1,0,3}, {PCA_ADDR_LOCAL_IO_1,0,5}, {PCA_ADDR_LOCAL_IO_1,0,7}, {PCA_ADDR_LOCAL_IO_1,1,1},
  {PCA_ADDR_LOCAL_IO_1,1,3}, {PCA_ADDR_LOCAL_IO_1,1,5}, {PCA_ADDR_LOCAL_IO_1,1,7}, {PCA_ADDR_LOCAL_IO_1,4,6}, {PCA_ADDR_LOCAL_IO_1,4,4},
  {PCA_ADDR_LOCAL_IO_1,4,2}, {PCA_ADDR_LOCAL_IO_1,4,0}, {PCA_ADDR_LOCAL_IO_1,3,6}, {PCA_ADDR_LOCAL_IO_1,3,4}, {PCA_ADDR_LOCAL_IO_1,3,2}
};

const PcaPin PowerPanel::relayCtrl_[HEATER_COUNT] =
{
  {PCA_ADDR_RELAY_CTRL,4,7}, {PCA_ADDR_RELAY_CTRL,4,6}, {PCA_ADDR_RELAY_CTRL,4,5}, {PCA_ADDR_RELAY_CTRL,4,4}, {PCA_ADDR_RELAY_CTRL,4,3},
  {PCA_ADDR_RELAY_CTRL,4,2}, {PCA_ADDR_RELAY_CTRL,4,1}, {PCA_ADDR_RELAY_CTRL,4,0}, {PCA_ADDR_RELAY_CTRL,3,7}, {PCA_ADDR_RELAY_CTRL,3,6},
  {PCA_ADDR_RELAY_CTRL,3,5}, {PCA_ADDR_RELAY_CTRL,3,4}, {PCA_ADDR_RELAY_CTRL,3,3}, {PCA_ADDR_RELAY_CTRL,3,2}, {PCA_ADDR_RELAY_CTRL,3,1},
  {PCA_ADDR_RELAY_CTRL,0,7}, {PCA_ADDR_RELAY_CTRL,1,0}, {PCA_ADDR_RELAY_CTRL,1,1}, {PCA_ADDR_RELAY_CTRL,1,2}, {PCA_ADDR_RELAY_CTRL,1,3},
  {PCA_ADDR_RELAY_CTRL,1,4}, {PCA_ADDR_RELAY_CTRL,1,5}, {PCA_ADDR_RELAY_CTRL,1,6}, {PCA_ADDR_RELAY_CTRL,0,0}, {PCA_ADDR_RELAY_CTRL,0,1},
  {PCA_ADDR_RELAY_CTRL,0,2}, {PCA_ADDR_RELAY_CTRL,0,3}, {PCA_ADDR_RELAY_CTRL,0,4}, {PCA_ADDR_RELAY_CTRL,0,5}, {PCA_ADDR_RELAY_CTRL,0,6}
};

const PcaPin PowerPanel::remoteI_[HEATER_COUNT] =
{
  {PCA_ADDR_REMOTE_I,0,0}, {PCA_ADDR_REMOTE_I,0,1}, {PCA_ADDR_REMOTE_I,0,2}, {PCA_ADDR_REMOTE_I,0,3}, {PCA_ADDR_REMOTE_I,0,4},
  {PCA_ADDR_REMOTE_I,0,5}, {PCA_ADDR_REMOTE_I,0,6}, {PCA_ADDR_REMOTE_I,0,7}, {PCA_ADDR_REMOTE_I,1,0}, {PCA_ADDR_REMOTE_I,1,1},
  {PCA_ADDR_REMOTE_I,1,2}, {PCA_ADDR_REMOTE_I,1,3}, {PCA_ADDR_REMOTE_I,1,4}, {PCA_ADDR_REMOTE_I,1,5}, {PCA_ADDR_REMOTE_I,1,6},
  {PCA_ADDR_REMOTE_I,4,7}, {PCA_ADDR_REMOTE_I,4,6}, {PCA_ADDR_REMOTE_I,4,5}, {PCA_ADDR_REMOTE_I,4,4}, {PCA_ADDR_REMOTE_I,4,3},
  {PCA_ADDR_REMOTE_I,4,2}, {PCA_ADDR_REMOTE_I,4,1}, {PCA_ADDR_REMOTE_I,4,0}, {PCA_ADDR_REMOTE_I,3,7}, {PCA_ADDR_REMOTE_I,3,6},
  {PCA_ADDR_REMOTE_I,3,5}, {PCA_ADDR_REMOTE_I,3,4}, {PCA_ADDR_REMOTE_I,3,3}, {PCA_ADDR_REMOTE_I,3,2}, {PCA_ADDR_REMOTE_I,3,1}
};

/**
 * ============================================================
 * Initialization
 * ============================================================
 */
void PowerPanel::begin(TwoWire& i2c0, TwoWire& i2c1)
{
  // Store I2C bus references
  _i2c0 = &i2c0;
  _i2c1 = &i2c1;

  // Configure ADC resolution for temperature reads
  analogReadResolution(ADC_RES_BITS);

  // Put relay PCA into known mode
  (void)writeRegs_(PCA_ADDR_RELAY_CTRL, PCA_REG_MODE_OUTPUTS_UPDATE_ON_STOP, &PCA_MODE_DEFAULT, 1);

  // Configure all relay pins as outputs
  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    const auto& p = relayCtrl_[i];
    (void)setPinDirection_(p.addr, p.bank, p.bit, false);
  }

  // Initialize all relays to OFF and sync state
  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    _state.heaters[i].relayCommand = false;

    const auto& p = relayCtrl_[i];
    (void)writeGpioBit_(p.addr, p.bank, p.bit, false);
  }

  // Initialize snapshot timestamp
  _state.snapshotMs = millis();
}

/**
 * ============================================================
 * I/O refresh (panel-wide)
 * ============================================================
 *
 * PCA9698 devices are bank-based, so refreshing I/O is always
 * a panel-level operation, not per-heater.
 */
bool PowerPanel::refreshIo()
{
  const bool localOk = refreshLocalIOBanks();
  const bool remoteOk = refreshRemoteIOBanks();

  // Snapshot timestamp represents "coherent state moment"
  _state.snapshotMs = millis();

  return localOk || remoteOk;
}

/**
 * ============================================================
 * Local I/O refresh
 * ============================================================
 *
 * Reads both LOCAL_IO PCA devices and distributes results
 * across all heaters.
 *
 * Includes:
 *   - local current (with debounce/presence logic)
 *   - local voltage (active-low normalized)
 */
bool PowerPanel::refreshLocalIOBanks()
{
  uint8_t ip0[5] = {0};
  uint8_t ip1[5] = {0};

  const bool ok0 = readRegs_(PCA_ADDR_LOCAL_IO_0, PCA_REG_IP0, ip0, 5);
  const bool ok1 = readRegs_(PCA_ADDR_LOCAL_IO_1, PCA_REG_IP0, ip1, 5);
  const bool anyOk = ok0 || ok1;

  _state.localIoValid = anyOk ? SignalValidity::VALID : SignalValidity::INVALID;
  if (!anyOk) return false;

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  _state.localIoUpdatedMs = nowMs;

  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    auto& hs = _state.heaters[i];
    bool haveLocalI = false;
    bool haveVolt = false;

    {
      const auto& p = localI_[i];
      const uint8_t* bank = nullptr;
      if (p.addr == PCA_ADDR_LOCAL_IO_0 && ok0)      bank = ip0;
      else if (p.addr == PCA_ADDR_LOCAL_IO_1 && ok1) bank = ip1;

      if (bank)
      {
        const bool active = (bank[p.bank] & (1u << p.bit)) != 0;
        auto& t = _localTrack[i];

        // Track edges and activity timing
        if (active != t.rawNow)
        {
          t.rawNow = active;
          t.lastChangeUs = nowUs;
        }

        // Record last time signal was active
        if (active)
          t.lastSeenActiveUs = nowUs;

        // Consider current "present" if seen recently
        t.present = (t.lastSeenActiveUs != 0) &&
                    ((nowUs - t.lastSeenActiveUs) < LOCAL_PRESENT_TIMEOUT_US);
        /**
         * Local current is derived from a comparator and may:
         *   - pulse
         *   - drop briefly
         *
         * So we convert raw signal into a "recently active"
         * presence indicator using a timeout window.
         */

        hs.localCurrent = t.present;
        haveLocalI = true;
      }
    }
    {
      const auto& p = volt_[i];
      const uint8_t* bank = nullptr;
      if (p.addr == PCA_ADDR_LOCAL_IO_0 && ok0)      bank = ip0;
      else if (p.addr == PCA_ADDR_LOCAL_IO_1 && ok1) bank = ip1;

      if (bank)
      {
        const bool bitSet = (bank[p.bank] & (1u << p.bit)) != 0;
        hs.localVoltage = !bitSet;
        haveVolt = true;
      }
    }

    if (haveLocalI && haveVolt)
    {
      hs.ioValid = SignalValidity::VALID;
      hs.ioUpdatedMs = nowMs;
      hs.stateMs = nowMs;
    }
    else
    {
      hs.ioValid = SignalValidity::INVALID;
    }
  }

  return true;
}

/**
 * ============================================================
 * Remote I/O refresh
 * ============================================================
 *
 * Reads remote current PCA once and updates all heaters.
 *
 * Remote current signal is:
 *   CT → rectifier → optocoupler → PCA input
 *   Active LOW at hardware → normalized to true = present
 */
bool PowerPanel::refreshRemoteIOBanks()
{
  uint8_t ip[5] = {0};
  const bool ok = readRegs_(PCA_ADDR_REMOTE_I, PCA_REG_IP0, ip, 5);

  _state.remoteIoValid = ok ? SignalValidity::VALID : SignalValidity::INVALID;
  if (!ok) return false;

  const uint32_t nowMs = millis();
  _state.remoteIoUpdatedMs = nowMs;

  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    auto& hs = _state.heaters[i];
    const auto& p = remoteI_[i];
    const bool bitSet = (ip[p.bank] & (1u << p.bit)) != 0;
    hs.remoteCurrent = !bitSet;

    if (hs.ioValid == SignalValidity::VALID)
      hs.ioUpdatedMs = nowMs;

    hs.stateMs = nowMs;
  }

  return true;
}

/**
 * ============================================================
 * Temperature acquisition
 * ============================================================
 *
 * Uses analog mux + ADC, so this is inherently per-channel.
 *
 * Flow:
 *   1. Select mux channel
 *   2. Allow settling
 *   3. Throw away first ADC sample
 *   4. Take real measurement
 *   5. Convert to °C
 */
bool PowerPanel::refreshTemp(uint8_t heaterIndex)
{
  if (heaterIndex >= HEATER_COUNT) return false;

  const uint8_t ch = heaterToTempMuxCh_(heaterIndex);
  tempMuxSelect_(ch);
  delayMicroseconds(TEMP_MUX_SETTLE_US);

  digitalWrite(PIN_T_MUX_EN, LOW);
  (void)analogRead(PIN_T_ADC);
  delayMicroseconds(TEMP_MUX_SETTLE_US);

  const uint16_t adc = analogRead(PIN_T_ADC);
  const float v = (adc / static_cast<float>(ADC_MAX_COUNTS)) * ADC_VREF;

  // MCP9700: 500mV offset, 10mV/°C slope
  const float tempC = (v - 0.500f) / 0.010f;

  auto& hs = _state.heaters[heaterIndex];
  hs.moduleTempC = tempC;
  hs.tempValid = SignalValidity::VALID;
  hs.tempUpdatedMs = millis();
  hs.stateMs = hs.tempUpdatedMs;
  _state.snapshotMs = hs.tempUpdatedMs;

  return true;
}

/**
 * @brief Round-robin temperature refresh
 *
 * Spreads temperature sampling across loop iterations
 * to avoid blocking on all 30 channels at once.
 */
bool PowerPanel::refreshNextTemp()
{
  const bool ok = refreshTemp(_nextTempIndex);
  _nextTempIndex = static_cast<uint8_t>((_nextTempIndex + 1u) % HEATER_COUNT);
  return ok;
}

/**
 * ============================================================
 * Relay control
 * ============================================================
 *
 * Writes PCA output and updates software state.
 *
 * NOTE:
 * This is command state, not feedback.
 */
bool PowerPanel::setRelay(uint8_t heaterIndex, bool on)
{
  if (heaterIndex >= HEATER_COUNT) return false;

  const auto& p = relayCtrl_[heaterIndex];
  if (!writeGpioBit_(p.addr, p.bank, p.bit, on))
    return false;

  auto& hs = _state.heaters[heaterIndex];
  hs.relayCommand = on;
  hs.stateMs = millis();
  _state.snapshotMs = hs.stateMs;
  return true;
}

bool PowerPanel::setRelayBanks(const uint8_t banks[5])
{
  if (!banks) return false;

  // Write OP0..OP4 in one transaction
  if (!writeRegs_(PCA_ADDR_RELAY_CTRL, PCA_REG_OP0, banks, 5))
    return false;

  const uint32_t now = millis();

  // Update software state from the same bank image
  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    const auto& p = relayCtrl_[i];
    const bool on = (banks[p.bank] & (1u << p.bit)) != 0;
    _state.heaters[i].relayCommand = on;
    _state.heaters[i].stateMs = now;
  }

  _state.snapshotMs = now;
  return true;
}

bool PowerPanel::setRelaysFromMask(uint32_t relayMask)
{
  uint8_t banks[5] = {0, 0, 0, 0, 0};

  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    const bool on = (relayMask & (1UL << i)) != 0;
    if (!on) continue;

    const auto& p = relayCtrl_[i];
    banks[p.bank] |= (1u << p.bit);
  }
  return setRelayBanks(banks);
}

/**
 * ============================================================
 * State access
 * ============================================================
 */
bool PowerPanel::localCurrentPresent(uint8_t heaterIndex) const
{
  if (heaterIndex >= HEATER_COUNT) return false;
  return _localTrack[heaterIndex].present;
}

HeaterState& PowerPanel::heaterState(uint8_t heaterIndex)
{
  static HeaterState invalid{};
  return (heaterIndex < HEATER_COUNT) ? _state.heaters[heaterIndex] : invalid;
}

const HeaterState* PowerPanel::heaterStates() const
{
  return _state.heaters;
}
/**
 * ============================================================
 * Low-level I2C helpers
 * ============================================================
 *
 * These wrap PCA register access and enforce:
 *   - correct bus selection
 *   - consistent read/write patterns
 */
TwoWire* PowerPanel::wireForAddr_(uint8_t addr) const
{
  if (addr == PCA_ADDR_LOCAL_IO_0 || addr == PCA_ADDR_LOCAL_IO_1) return _i2c0;
  if (addr == PCA_ADDR_RELAY_CTRL || addr == PCA_ADDR_REMOTE_I)   return _i2c1;
  return nullptr;
}

uint8_t PowerPanel::heaterToTempMuxCh_(uint8_t heaterIndex)
{
  if (heaterIndex < 15) return heaterIndex;
  return static_cast<uint8_t>(heaterIndex + 1);
}

void PowerPanel::tempMuxSelect_(uint8_t ch)
{
  ch &= 0x1F;

  digitalWrite(PIN_T_MUX_CS, LOW);
  digitalWrite(PIN_T_MUX_WR, LOW);
  digitalWrite(PIN_T_MUX_EN, LOW);

  digitalWrite(PIN_T_MUX_A0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_T_MUX_A1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_T_MUX_A2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_T_MUX_A3, (ch & 0x08) ? HIGH : LOW);
  digitalWrite(PIN_T_MUX_A4, (ch & 0x10) ? HIGH : LOW);

  digitalWrite(PIN_T_MUX_EN, HIGH);
  digitalWrite(PIN_T_MUX_WR, HIGH);
  digitalWrite(PIN_T_MUX_CS, HIGH);
}

bool PowerPanel::readRegs_(uint8_t addr, uint8_t startReg, uint8_t* dst, uint8_t count) const
{
  TwoWire* w = wireForAddr_(addr);
  if (!w || !dst || count == 0) return false;

  // Auto-increment bit enables sequential register reads
  const uint8_t cmd = startReg | 0x80;   // PCA9698 auto-increment flag (bit 7) allows multi-byte reads
  w->beginTransmission(addr);
  w->write(cmd);
  if (w->endTransmission(false) != 0)
    return false;

  const uint8_t n = w->requestFrom(static_cast<int>(addr), static_cast<int>(count));
  if (n != count)
  {
    while (w->available()) (void)w->read();
    return false;
  }

  for (uint8_t i = 0; i < count; ++i)
    dst[i] = w->read();

  return true;
}

bool PowerPanel::writeRegs_(uint8_t addr, uint8_t startReg,
                            const uint8_t* src, uint8_t count) const
{
  TwoWire* w = wireForAddr_(addr);
  if (!w || !src || count == 0) return false;

  const uint8_t cmd = startReg | 0x80;   // auto-increment
  w->beginTransmission(addr);
  w->write(cmd);

  for (uint8_t i = 0; i < count; ++i)
    w->write(src[i]);

  return (w->endTransmission() == 0);
}

bool PowerPanel::setPinDirection_(uint8_t addr, uint8_t port, uint8_t bit, bool isInput) const
{
  if (port > 4 || bit > 7) return false;

  const uint8_t reg = PCA_REG_IOC0 + port;
  uint8_t cur = 0;
  if (!readRegs_(addr, reg, &cur, 1))
    return false;

  if (isInput) cur |=  (1u << bit);
  else         cur &= ~(1u << bit);

  return writeRegs_(addr, reg, &cur, 1);
}

bool PowerPanel::readGpioBit_(uint8_t addr, uint8_t port, uint8_t bit, bool& stateOut) const
{
  if (port > 4 || bit > 7) return false;

  const uint8_t reg = PCA_REG_IP0 + port;
  uint8_t value = 0;
  if (!readRegs_(addr, reg, &value, 1))
    return false;

  stateOut = ((value >> bit) & 0x01) != 0;
  return true;
}

bool PowerPanel::writeGpioBit_(uint8_t addr, uint8_t port, uint8_t bit, bool value) const
{
  if (port > 4 || bit > 7) return false;

  const uint8_t reg = PCA_REG_OP0 + port;
  uint8_t cur = 0;
  if (!readRegs_(addr, reg, &cur, 1))
    return false;

  if (value) cur |=  (1u << bit);
  else       cur &= ~(1u << bit);

  return writeRegs_(addr, reg, &cur, 1);
}
