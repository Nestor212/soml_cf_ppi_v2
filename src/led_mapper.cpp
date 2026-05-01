#include "led_mapper.hpp"

/// Clear all LED intensities in frame.
void LedPanelMapper::clear(ScanFrame& frame)
{
  frame = {};
}

/// Map heater index to LED column (0-14 for both left/right banks).
uint8_t LedPanelMapper::colFor_(uint8_t heaterIndex)
{
  return (heaterIndex < PANEL_COL_COUNT)
           ? heaterIndex
           : static_cast<uint8_t>(heaterIndex - PANEL_COL_COUNT);
}

/// Map heater index to LED bank (0 for heaters 0-14, 1 for 15-29).
uint8_t LedPanelMapper::bankFor_(uint8_t heaterIndex)
{
  return (heaterIndex < PANEL_COL_COUNT) ? 0 : 1;
}

/// Scan line for fault status (line 0 or 1 by bank).
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

/// Fault condition: invalid I/O or relay/feedback mismatch.
bool LedPanelMapper::computeFault_(const HeaterState& st) const
{
  // Invalid I/O => fault
  if (st.ioValid != SignalValidity::VALID)
    return true;

  // Relay ON: expect all feedbacks
  if (st.relayCommand)
  {
    if (!st.localVoltage)  return true;
    if (!st.localCurrent)  return true;
    if (!st.remoteCurrent) return true;
  }
  // Relay OFF: expect no feedbacks
  if (!st.relayCommand)
  {
    if (st.localVoltage)  return true;
    if (st.localCurrent)  return true;
    if (st.remoteCurrent) return true;
  }
  return false;
}

/// Render single heater into frame (fault=RED, others=GREEN).
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

/// Render all heaters into frame (clear then populate).
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