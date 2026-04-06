#include "led_mapper.hpp"

/*
 * Clear the entire scan frame.
 */
void LedPanelMapper::clear(ScanFrame& frame)
{
  frame = {};
}

/*
 * Heater 0..14 map to column 0..14 on the "left" bank.
 * Heater 15..29 map to column 0..14 on the "right" bank.
 */
uint8_t LedPanelMapper::colFor_(uint8_t heaterIndex)
{
  return (heaterIndex < PANEL_COL_COUNT)
           ? heaterIndex
           : static_cast<uint8_t>(heaterIndex - PANEL_COL_COUNT);
}

/*
 * Bank 0 = heaters 0..14
 * Bank 1 = heaters 15..29
 */
uint8_t LedPanelMapper::bankFor_(uint8_t heaterIndex)
{
  return (heaterIndex < PANEL_COL_COUNT) ? 0 : 1;
}

/*
 * Scan-line lookup based on the actual frontplane wiring.
 */
uint8_t LedPanelMapper::faultLineFor_(uint8_t heaterIndex)
{
  return (bankFor_(heaterIndex) == 0) ? 0 : 1;
}

uint8_t LedPanelMapper::relayLineFor_(uint8_t heaterIndex)
{
  return (bankFor_(heaterIndex) == 0) ? 0 : 1;
}

uint8_t LedPanelMapper::localVLineFor_(uint8_t heaterIndex)
{
  return (bankFor_(heaterIndex) == 0) ? 2 : 3;
}

uint8_t LedPanelMapper::localILineFor_(uint8_t heaterIndex)
{
  return (bankFor_(heaterIndex) == 0) ? 4 : 5;
}

uint8_t LedPanelMapper::remoteILineFor_(uint8_t heaterIndex)
{
  return (bankFor_(heaterIndex) == 0) ? 6 : 7;
}

/*
 * Conservative bring-up fault logic.
 *
 * This keeps the same meaning you had before:
 * - invalid IO => fault
 * - if relay is commanded ON, the expected feedbacks should also be present
 */
bool LedPanelMapper::computeFault_(const HeaterState& st) const
{
  if (st.ioValid != SignalValidity::VALID)
    return true;

  if (st.relayCommand)
  {
    if (!st.localVoltage)  return true;
    if (!st.localCurrent)  return true;
    if (!st.remoteCurrent) return true;
  }
  if(!st.relayCommand)
  {
    if (st.localVoltage)  return true;
    if (st.localCurrent)  return true;
    if (st.remoteCurrent) return true;
  }
  return false;
}

/*
 * Convert one HeaterState into the real scan-frame representation.
 *
 * Important:
 * - fault uses RED only on line 0/1
 * - all other indicators use GREEN on their assigned lines
 */
void LedPanelMapper::renderOne(uint8_t heaterIndex,
                               const HeaterState& st,
                               ScanFrame& frame)
{
  if (heaterIndex >= PANEL_HEATER_COUNT)
    return;

  const uint8_t col = colFor_(heaterIndex);

  frame.r[faultLineFor_(heaterIndex)][col] =
      computeFault_(st) ? RED_ON : OFF;

  frame.g[relayLineFor_(heaterIndex)][col] =
      st.relayCommand ? GREEN_ON : OFF;

  frame.g[localVLineFor_(heaterIndex)][col] =
      st.localVoltage ? GREEN_ON : OFF;

  frame.g[localILineFor_(heaterIndex)][col] =
      st.localCurrent ? GREEN_ON : OFF;

  frame.g[remoteILineFor_(heaterIndex)][col] =
      st.remoteCurrent ? GREEN_ON : OFF;
}

/*
 * Convert all heater states into the scan frame.
 */
void LedPanelMapper::renderAll(const HeaterState* states,
                               uint8_t count,
                               ScanFrame& frame)
{
  clear(frame);

  if (!states)
    return;

  const uint8_t n =
      (count < PANEL_HEATER_COUNT) ? count : PANEL_HEATER_COUNT;

  for (uint8_t i = 0; i < n; ++i)
    renderOne(i, states[i], frame);
}