#pragma once

#include <Arduino.h>
#include "led_panel_types.hpp"
#include "heaterState.hpp"

class LedPanelMapper
{
public:
  static constexpr uint16_t RED_ON   = PANEL_GS_MAX;
  static constexpr uint16_t GREEN_ON = PANEL_GS_MAX;
  static constexpr uint16_t OFF      = 0;

  void clear(ScanFrame& frame);
  void renderOne(uint8_t heaterIndex, const HeaterState& st, ScanFrame& frame);
  void renderAll(const HeaterState* states, uint8_t count, ScanFrame& frame);

private:
  static uint8_t colFor_(uint8_t heaterIndex);
  static uint8_t bankFor_(uint8_t heaterIndex);

  static uint8_t faultLineFor_(uint8_t heaterIndex);
  static uint8_t relayLineFor_(uint8_t heaterIndex);
  static uint8_t localVLineFor_(uint8_t heaterIndex);
  static uint8_t localILineFor_(uint8_t heaterIndex);
  static uint8_t remoteILineFor_(uint8_t heaterIndex);

  bool computeFault_(const HeaterState& st) const;
};