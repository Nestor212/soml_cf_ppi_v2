#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "ppi_global.hpp"
#include "powerPanel.hpp"
#include "LP5899_driver.hpp"
#include "led_mapper.hpp"
#include "og_ppi.hpp"

/**
 * ============================================================
 * Global system objects
 * ============================================================
 */

LP5899_PanelDriver ledDriver(PIN_PANEL_CS, PIN_PANEL_DRDY, PIN_PANEL_FAULT, SPI);
LedPanelMapper ledMapper;
ScanFrame panelFrame;

// Main system controller
PowerPanel powerPanel;
PowerPanelInterface ppi;
Threads::Mutex networkLock;

static IPAddress ntpServerIp[NUM_BROKERS] = {
  IPAddress(MQTT_BROKER_1),
  IPAddress(MQTT_BROKER_2)
};

// Timing helpers
static elapsedMillis tempPollMs; // temperature polling scheduler
static elapsedMicros ioPollUs;   // I/O polling scheduler
static elapsedMicros vsyncUs;    // LP5890 VSYNC keepalive scheduler
static elapsedMillis panelMs;    // LED frame update scheduler
static elapsedMillis loopHzMs;    // main-loop frequency measurement window
static elapsedMillis relayToggleMs; // relay toggle scheduler
static elapsedMillis printStatusMs; // status print scheduler
static elapsedMillis networkPollMs; // network/MQTT polling scheduler

static constexpr bool TOGGLE_RELAYS = true;
static constexpr uint32_t DEBUG_RELAY_MASK =
  (1UL << 24) |
  (1UL << 27) |
  (1UL << 28);

static constexpr uint32_t PANEL_VSYNC_PERIOD_US = 2000; // 500 Hz
static constexpr uint32_t PANEL_FRAME_PERIOD_MS = 50;  // 50 Hz mapped panel refresh
static constexpr uint32_t RELAY_TOGGLE_PERIOD_MS = 250;
static constexpr uint32_t STATUS_PRINT_PERIOD_MS = 100;
static constexpr uint32_t NETWORK_POLL_PERIOD_MS = 100;
static constexpr bool PRINT_ALL_HEATER_STATUS = false;
static constexpr bool PANEL_REQUIRE_DRDY_BEFORE_UPDATE = true;

static bool networkInitOkay = false;

static void applyPanelRuntimePreamble_()
{
  // Match the known-good startup sequence that reliably enables panel output.
  (void)ledDriver.debugWriteRegister(0x02, 0x000D); // CCSICTRL nominal forwarding
  (void)ledDriver.debugWriteRegister(0x05, 0x0001); // DEVCTRL: EXIT_FS
  delay(2);
  (void)ledDriver.debugWriteRegister(0x05, 0x0000);
}

static uint8_t computeFaultForPrint_(const HeaterState& st)
{
  if (st.ioValid == SignalValidity::INVALID)
  {
    return 1;
  }

  if (st.relayCommand)
  {
    if (!st.localVoltage || !st.localCurrent || !st.remoteCurrent)
    {
      return 1;
    }
  }
  else if (st.localVoltage || st.localCurrent || st.remoteCurrent)
  {
    return 1;
  }

  return 0;
}

static uint32_t setRelayMask_(bool& on)
{
  if (TOGGLE_RELAYS)
  {
    on = !on;
    return on ? DEBUG_RELAY_MASK : 0UL;
  }

  return 0UL;
}

static bool uploadPanelFrame_()
{
  ledMapper.renderAll(powerPanel.heaterStates(), HEATER_COUNT, panelFrame);
  ledDriver.loadFrame(panelFrame);
  const bool updated = ledDriver.update(PANEL_REQUIRE_DRDY_BEFORE_UPDATE);
  if (updated)
  {
    return true;
  }

  Serial.println(F("Panel update failed; status follows:"));
  ledDriver.printPins(Serial);
  (void)ledDriver.debugReadStatus(&Serial);
  return false;
}

static void printHeaterStatus_()
{
  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    const auto& state = powerPanel.heaterState(i);

    Serial.print("H"); Serial.print(i);
    Serial.print(" temp="); Serial.print(state.moduleTempC);
    Serial.print(" relay="); Serial.print(state.relayCommand);
    Serial.print(" V="); Serial.print(state.localVoltage);
    Serial.print(" LI="); Serial.print(state.localCurrent);
    Serial.print(" RI="); Serial.print(state.remoteCurrent);
    Serial.print(" ioValid="); Serial.print((uint8_t)state.ioValid);
    Serial.print(" faultPin="); Serial.println(computeFaultForPrint_(state));
  }
  Serial.println("===============================");
}

/**
 * @brief Check if an I2C device responds at a given address.
 */
bool probeDevice(TwoWire &bus, uint8_t addr)
{
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

/**
 * @brief Print presence of a device on a bus.
 */
bool checkDevice(TwoWire &bus, uint8_t addr, const char *busname)
{
  Serial.print("Checking ");
  Serial.print(busname);
  Serial.print(" addr 0x");
  Serial.print(addr, HEX);
  Serial.print(" ... ");

  const bool ok = probeDevice(bus, addr);
  Serial.println(ok ? "FOUND" : "NOT FOUND");
  return ok;
}

/**
 * ============================================================
 * Temperature mux setup
 * ============================================================
 *
 * Configures GPIO pins used to select analog mux channel.
 */
void tempMuxBegin_()
{
  pinMode(PIN_T_MUX_EN, OUTPUT);
  pinMode(PIN_T_MUX_WR, OUTPUT);
  pinMode(PIN_T_MUX_CS, OUTPUT);
  pinMode(PIN_T_MUX_A0, OUTPUT);
  pinMode(PIN_T_MUX_A1, OUTPUT);
  pinMode(PIN_T_MUX_A2, OUTPUT);
  pinMode(PIN_T_MUX_A3, OUTPUT);
  pinMode(PIN_T_MUX_A4, OUTPUT);

  // ADC input with pulldown for stability
  pinMode(PIN_T_ADC, INPUT_PULLDOWN);

  // Default mux state
  digitalWrite(PIN_T_MUX_EN, LOW);
  digitalWrite(PIN_T_MUX_CS, LOW);
  digitalWrite(PIN_T_MUX_WR, HIGH);
}

/**
 * ============================================================
 * Setup
 * ============================================================
 */
void setup()
{
  Serial.begin(460800);
  while (!Serial && millis() < 2000) {}
  delay(2000);

  Serial.println("Booting heater + panel system...");

  /**
   * ---- PP Address jumper pins ----
   */
  pinMode(JUMPER_PIN_1, INPUT_PULLUP);
  pinMode(JUMPER_PIN_2, INPUT_PULLUP);
  pinMode(JUMPER_PIN_3, INPUT_PULLUP);
  pinMode(JUMPER_PIN_4, INPUT_PULLUP);
  pinMode(JUMPER_PIN_5, INPUT_PULLUP);
  delay(10); // allow time for pullups to stabilize

  int jumpers = (int)digitalRead(JUMPER_PIN_1);
  jumpers += ((int)digitalRead(JUMPER_PIN_2)) << 1;
  jumpers += ((int)digitalRead(JUMPER_PIN_3)) << 2;
  jumpers += ((int)digitalRead(JUMPER_PIN_4)) << 3;
  jumpers += ((int)digitalRead(JUMPER_PIN_5)) << 4;
  Serial.println("Hardware ID: " + String(jumpers, BIN));

  /*
   * LED panel bring-up
   */
  ledDriver.begin(PANEL_SPI_BRINGUP_HZ);
  if (!ledDriver.init())
  {
    Serial.println(F("LP5899/LP5890 init failed"));
  }
  // Serial.println();
  // Serial.println(F("===== LP5899 BASIC TEST ====="));
  // ledDriver.printPins(Serial);
  // ledDriver.debugReadDeviceId(&Serial);
  // ledDriver.debugReadStatus(&Serial);
  // applyPanelRuntimePreamble_();

  // Serial.println(F("============================="));
  // Serial.print(F(" vsync_us="));
  // Serial.print(PANEL_VSYNC_PERIOD_US);
  // Serial.print(F(" frame_ms="));
  // Serial.println(PANEL_FRAME_PERIOD_MS);

  /**
   * ---- PCA reset + interrupt pins ----
   * Ensure all PCA chips are released from reset and
   * interrupt lines are properly pulled.
   */
  pinMode(PCA_RST_LOCAL_IO_0, OUTPUT);
  digitalWrite(PCA_RST_LOCAL_IO_0, HIGH);
  pinMode(PCA_INT_LOCAL_IO_0, INPUT_PULLUP);

  pinMode(PCA_RST_LOCAL_IO_1, OUTPUT);
  digitalWrite(PCA_RST_LOCAL_IO_1, HIGH);
  pinMode(PCA_INT_LOCAL_IO_1, INPUT_PULLUP);

  pinMode(PCA_RST_RELAY_CTRL, OUTPUT);
  digitalWrite(PCA_RST_RELAY_CTRL, HIGH);
  pinMode(PCA_INT_RELAY_CTRL, INPUT_PULLUP);

  pinMode(PCA_RST_REMOTE_I, OUTPUT);
  digitalWrite(PCA_RST_REMOTE_I, HIGH);
  pinMode(PCA_INT_REMOTE_I, INPUT_PULLUP);

  /**
   * ---- I2C initialization ----
   */
  Wire.setClock(I2C_HZ);
  Wire.begin();

  Wire1.setClock(I2C_HZ);
  Wire1.begin();

  // Allow devices time to power up
  delay(1000);

  /**
   * ---- Device presence check ----
   */
  checkDevice(Wire,  PCA_ADDR_LOCAL_IO_0, "Wire");
  checkDevice(Wire,  PCA_ADDR_LOCAL_IO_1, "Wire");
  checkDevice(Wire1, PCA_ADDR_RELAY_CTRL, "Wire1");
  checkDevice(Wire1, PCA_ADDR_REMOTE_I,   "Wire1");

  /**
   * ---- Initialize subsystems ----
   */
  tempMuxBegin_();
  powerPanel.begin(Wire, Wire1);

  /**
   * ---- Attach heater wrappers ----
   * Each Heater becomes a view into PowerPanel state.
   */
  for (uint8_t i = 0; i < HEATER_COUNT; ++i)
  {
    // Ensure relays start OFF
    powerPanel.setRelay(i, false);
    // Prime temperature readings
    (void)powerPanel.refreshTemp(i);
  }

  // Prime I/O state
  (void)powerPanel.refreshIo();
  (void)uploadPanelFrame_();

  /**
   * ---- Network/MQTT/NTP initialization ----
   */
  Serial.println();
  Serial.println(F("Initializing PowerPanel network interface..."));
  ppi.attach_panel(&powerPanel);
  (void)ppi.start_panel(PPI_JP5 ? 2 : 1);
  networkInitOkay = ppi.network_init(&networkLock);
  if (!networkInitOkay)
  {
    Serial.println(F("WARNING: network_init failed (continuing without broker/NTP)"));
  }
  else
  {
    delay(1000);
    if (!ntp_start(ntpServerIp, NUM_ELEM(ntpServerIp), &networkLock))
    {
      Serial.println(F("WARNING: ntp_start failed"));
    }
  }
  Serial.println();

  Serial.println("Setup complete.");
}

/**
 * ============================================================
 * Main loop
 * ============================================================
 *
 * Uses non-blocking timing (elapsedMillis / elapsedMicros)
 * to schedule different subsystems independently.
 */
void loop()
{
  static bool relayMaskOn = false;

  (void)powerPanel.schedulerTick(ntp_get_current_time_millis());

  if (relayToggleMs >= RELAY_TOGGLE_PERIOD_MS)
  {
    relayToggleMs = 0;

    const uint32_t mask = setRelayMask_(relayMaskOn);
    powerPanel.setRelaysFromMask(mask);
  }

  /**
   * ---- I/O polling (PCA) (fast) ----
   * Reads all heater digital inputs in one batch.
   */
  if (ioPollUs >= IO_POLL_INTERVAL_US)
  {
    ioPollUs = 0;
    (void)powerPanel.refreshIo();
  }

  /**
   * ---- Temperature polling (slow)----
   * Round-robin updates one heater at a time.
   */
  if (tempPollMs >= TEMP_POLL_INTERVAL_MS)
  {
    tempPollMs = 0;
    (void)powerPanel.refreshNextTemp();
  }

  if (networkInitOkay && networkPollMs >= NETWORK_POLL_PERIOD_MS)
  {
    networkPollMs = 0;
    (void)ppi.update_status(false);
    (void)ppi.check_broker();
  }

  if (panelMs >= PANEL_FRAME_PERIOD_MS)
  {
    panelMs = 0;
    (void)uploadPanelFrame_();
    if (PRINT_ALL_HEATER_STATUS) printHeaterStatus_();
  }
}

