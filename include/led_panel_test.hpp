#pragma once

#include <Arduino.h>
#include "led_panel_types.hpp"
#include "LP5899_driver.hpp"

enum class PanelTestRow : uint8_t
{
  FaultLeft = 0,
  FaultRight,
  RelayLeft,
  RelayRight,
  LocalVLeft,
  LocalVRight,
  LocalILeft,
  LocalIRight,
  RemoteILeft,
  RemoteIRight
};

void panelTestClear(ScanFrame& frame);
void panelTestAllOff(ScanFrame& frame, LP5899_PanelDriver& driver, bool pushToHardware);
void panelTestAllOn(ScanFrame& frame, LP5899_PanelDriver& driver, bool pushToHardware);

void panelTestSetFault(ScanFrame& frame, uint8_t heater, uint16_t value);
void panelTestSetRelay(ScanFrame& frame, uint8_t heater, uint16_t value);
void panelTestSetLocalV(ScanFrame& frame, uint8_t heater, uint16_t value);
void panelTestSetLocalI(ScanFrame& frame, uint8_t heater, uint16_t value);
void panelTestSetRemoteI(ScanFrame& frame, uint8_t heater, uint16_t value);

void panelTestSingleFault(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware);  
void panelTestSingleRelay(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware);
void panelTestSingleLocalV(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware);  
void panelTestSingleLocalI(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware);  
void panelTestSingleRemoteI(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware); 

void panelTestSingleHeaterAll(ScanFrame& frame,
                          LP5899_PanelDriver& driver,
                          uint8_t heater,
                          bool pushToHardware);                           

void panelTestShow(const ScanFrame& frame,
                   LP5899_PanelDriver& driver,
                   bool pushToHardware);