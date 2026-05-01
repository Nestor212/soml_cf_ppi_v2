#include "led_panel_test.hpp"

namespace
{
  constexpr uint16_t LED_ON  = PANEL_GS_MAX;
  constexpr uint16_t LED_OFF = 0;

  uint8_t colForHeater(uint8_t heater)
  {
    return (heater < 15) ? heater : static_cast<uint8_t>(heater - 15);
  }

  uint8_t bankForHeater(uint8_t heater)
  {
    return (heater < 15) ? 0 : 1;
  }
}

/*
 * Clear the whole scan frame.
 */
void panelTestClear(ScanFrame& frame)
{
  frame = {};
}

/*
 * Set every used scan location ON.
 *
 * Note:
 * - red is only used on lines 0 and 1 for fault
 * - green is used on all 8 lines
 */
void panelTestAllOn(ScanFrame& frame,
                    LP5899_PanelDriver& driver,
                    bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: ALL ON ====="));

  panelTestClear(frame);

  for (uint8_t col = 0; col < PANEL_COL_COUNT; ++col)
  {
    frame.r[0][col] = LED_ON;
    frame.r[1][col] = LED_ON;

    for (uint8_t line = 0; line < PANEL_LINE_COUNT; ++line)
      frame.g[line][col] = LED_ON;
  }

  panelTestShow(frame, driver, pushToHardware);
}

/*
 * Set every used scan location OFF.
 */
void panelTestAllOff(ScanFrame& frame,
                     LP5899_PanelDriver& driver,
                     bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: ALL OFF ====="));

  panelTestClear(frame);
  panelTestShow(frame, driver, pushToHardware);
}

/*
 * Individual logical setters that match your real panel wiring.
 */
void panelTestSetFault(ScanFrame& frame, uint8_t heater, uint16_t value)
{
  if (heater >= PANEL_HEATER_COUNT)
    return;

  const uint8_t col  = colForHeater(heater);
  const uint8_t line = (bankForHeater(heater) == 0) ? 0 : 1;

  frame.r[line][col] = value;
}

void panelTestSetRelay(ScanFrame& frame, uint8_t heater, uint16_t value)
{
  if (heater >= PANEL_HEATER_COUNT)
    return;

  const uint8_t col  = colForHeater(heater);
  const uint8_t line = (bankForHeater(heater) == 0) ? 0 : 1;

  frame.g[line][col] = value;
}

void panelTestSetLocalV(ScanFrame& frame, uint8_t heater, uint16_t value)
{
  if (heater >= PANEL_HEATER_COUNT)
    return;

  const uint8_t col  = colForHeater(heater);
  const uint8_t line = (bankForHeater(heater) == 0) ? 2 : 3;

  frame.g[line][col] = value;
}

void panelTestSetLocalI(ScanFrame& frame, uint8_t heater, uint16_t value)
{
  if (heater >= PANEL_HEATER_COUNT)
    return;

  const uint8_t col  = colForHeater(heater);
  const uint8_t line = (bankForHeater(heater) == 0) ? 4 : 5;

  frame.g[line][col] = value;
}

void panelTestSetRemoteI(ScanFrame& frame, uint8_t heater, uint16_t value)
{
  if (heater >= PANEL_HEATER_COUNT)
    return;

  const uint8_t col  = colForHeater(heater);
  const uint8_t line = (bankForHeater(heater) == 0) ? 6 : 7;

  frame.g[line][col] = value;
}

/*
 * Show one fault LED in the scan frame.
 */
void panelTestSingleFault(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: SINGLE FAULT ====="));
  Serial.print(F("heater="));
  Serial.println(heater);

  panelTestClear(frame);
  panelTestSetFault(frame, heater, LED_ON);
  panelTestShow(frame, driver, pushToHardware);
}

/*
 * Show one relay LED in the scan frame.
 */
void panelTestSingleRelay(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: SINGLE RELAY ====="));
  Serial.print(F("heater="));
  Serial.println(heater);

  panelTestClear(frame);
  panelTestSetRelay(frame, heater, LED_ON);
  panelTestShow(frame, driver, pushToHardware);
}

/*
 * Show one Local V LED in the scan frame.
 */
void panelTestSingleLocalV(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: SINGLE LOCAL V ====="));
  Serial.print(F("heater="));
  Serial.println(heater);

  panelTestClear(frame);
  panelTestSetLocalV(frame, heater, LED_ON);
  panelTestShow(frame, driver, pushToHardware);
}

/*
 * Show one Local I LED in the scan frame.
 */
void panelTestSingleLocalI(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: SINGLE LOCAL I ====="));
  Serial.print(F("heater="));
  Serial.println(heater);

  panelTestClear(frame);
  panelTestSetLocalI(frame, heater, LED_ON);
  panelTestShow(frame, driver, pushToHardware);
}                          

/*
 * Show one Remote I LED in the scan frame.
 */
void panelTestSingleRemoteI(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: SINGLE REMOTE I ====="));
  Serial.print(F("heater="));
  Serial.println(heater);

  panelTestClear(frame);
  panelTestSetRemoteI(frame, heater, LED_ON);
  panelTestShow(frame, driver, pushToHardware);
}

/*
 * Show all LEDs for one heater in the scan frame.
 */
void panelTestSingleHeaterAll(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware)
{
  Serial.println();
  Serial.println(F("===== PANEL TEST: SINGLE HEATER ALL ====="));
  Serial.print(F("heater="));
  Serial.println(heater);

  panelTestClear(frame);
  panelTestSetFault(frame, heater, LED_ON);
  panelTestSetRelay(frame, heater, LED_ON);
  panelTestSetLocalV(frame, heater, LED_ON);
  panelTestSetLocalI(frame, heater, LED_ON);
  panelTestSetRemoteI(frame, heater, LED_ON);  
  panelTestShow(frame, driver, pushToHardware);
}                         

/*
 * Debug/show helper.
 *
 * At this stage the most valuable thing is to confirm:
 * - scan frame contents are correct
 * - LP5899 status pins look sane
 * - SPI activity is occurring
 *
 * The actual LP5899/LP5890 protocol is still not complete in the driver.
 */
void panelTestShow(const ScanFrame& frame,
                   LP5899_PanelDriver& driver,
                   bool pushToHardware)
{
  driver.loadFrame(frame);
  driver.dumpScanFrame(Serial);
  driver.printPins(Serial);

  if (pushToHardware)
  {
    const bool ok = driver.update(false);
    Serial.println(ok ? F("update() done") : F("update() failed"));
  }
}