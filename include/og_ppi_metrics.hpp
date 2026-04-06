#ifndef __PPI_METRICS_HPP__
#define __PPI_METRICS_HPP__
/*******************************************************************************
Copyright 2023
Steward Observatory Engineering & Technical Services, University of Arizona

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*******************************************************************************/

/**
@brief SparkplugB Metrics for the SOML Casting Furnace Power Panel Interface
@author Michael Sibayan
@date June 6, 2023
@file ppi_metrics.hpp

This file contains the SparkplugB metric definitions for the PPI.
*/


// Overall version of the MQTT messages.  Increment this for any change to
// the messages: added, deleted, renamed, different type, different function.
#define PPI_COMMS_VERSION  6

// ID components of the MQTT SparkplugB topics used by this node
#define PPI_GROUP_ID  "PS"
#define PPI_NODE_ID   "PPI"

// Use namespace to avoid conflicts with metric definitions for other modules
namespace PPI
{
  // Alias numbers for each of the node metrics
  typedef enum {
    NMA_bdSeq = 0,
    NMA_Reboot,
    NMA_Rebirth,
    NMA_NextServer,
    NMA_CommsVersion,
    NMA_FirmwareVersion,
    NMA_UniqueId,
    NMA_CommsChannel,
    NMA_NTPSynchronized,
    NMA_NTPLastUpdate,
    NMA_NTPServer,
    NMA_PanelStatus,
    NMA_ActiveSchedule,
    NMA_ActiveScheduleStart,
    NMA_CommandCounter,
    NMA_CommandIndex,
    NMA_CommandSentNTP,
    NMA_CommandSentMicros,
    NMA_HeaterOutput,
    NMA_VMon,
    NMA_ILMon,
    NMA_IRMon,
    NMA_ErrorLamps,
    NMA_CyclePeriod,
    NMA_CycleOffset,
    NMA_NextSchedule,
    NMA_NextScheduleStart,
    NMA_VDown,
    NMA_ILDown,
    NMA_IRDown,
#if defined(SIMULATED_POWER_PANEL)
    NMA_Sim_NoReply,
    NMA_Sim_ParityErr,
    NMA_Sim_InvalidHex,
    NMA_Sim_ChecksumErr,
    NMA_Sim_NoStrobe,
    NMA_Sim_ShortReply,
    NMA_Sim_LongReply,
    NMA_Sim_VOn,
    NMA_Sim_VOff,
    NMA_Sim_ILOn,
    NMA_Sim_ILOff,
    NMA_Sim_IROn,
    NMA_Sim_IROff,
    NMA_Sim_ClockOffset,
#endif
    EndNodeMetricAlias
  } node_metric_alias_t;

  // Node metric names
  static const char *MetricNames[EndNodeMetricAlias] = {
    [NMA_bdSeq]               = "bdSeq",
    [NMA_Reboot]              = "Node Control/Reboot",
    [NMA_Rebirth]             = "Node Control/Rebirth",
    [NMA_NextServer]          = "Node Control/Next Server",
    [NMA_CommsVersion]        = "Properties/Communications Version",
    [NMA_FirmwareVersion]     = "Properties/Firmware Version",
    [NMA_UniqueId]            = "Properties/Unique Id",
    [NMA_CommsChannel]        = "Properties/Communications Channel",
    [NMA_NTPSynchronized]     = "Inputs/NTP Synchronized",
    [NMA_NTPLastUpdate]       = "Inputs/NTP Last Update Time",
    [NMA_NTPServer]           = "Inputs/NTP Server",
    [NMA_PanelStatus]         = "Inputs/Panel Status",
    [NMA_ActiveSchedule]      = "Inputs/Active Schedule",
    [NMA_ActiveScheduleStart] = "Inputs/Active Schedule Start",
    [NMA_CommandCounter]      = "Inputs/Command Counter",
    [NMA_CommandIndex]        = "Inputs/Command Index",
    [NMA_CommandSentNTP]      = "Inputs/Command Sent NTP",
    [NMA_CommandSentMicros]   = "Inputs/Command Sent Micros",
    [NMA_HeaterOutput]        = "Inputs/Heater Output",
    [NMA_VMon]                = "Inputs/Voltage Monitor",
    [NMA_ILMon]               = "Inputs/Local Current Monitor",
    [NMA_IRMon]               = "Inputs/Remote Current Monitor",
    [NMA_ErrorLamps]          = "Inputs/Error Lamps",
    [NMA_CyclePeriod]         = "Inputs/Cycle Period",
    [NMA_CycleOffset]         = "Inputs/Cycle NTP Offset",
    [NMA_NextSchedule]        = "Outputs/Next Schedule",
    [NMA_NextScheduleStart]   = "Outputs/Next Schedule Start",
    [NMA_VDown]               = "Outputs/Voltage Sensor Down",
    [NMA_ILDown]              = "Outputs/Local Current Sensor Down",
    [NMA_IRDown]              = "Outputs/Remote Current Sensor Down",
#if defined(SIMULATED_POWER_PANEL)
    [NMA_Sim_NoReply]         = "Simulation/No Reply",
    [NMA_Sim_ParityErr]       = "Simulation/Parity Error",
    [NMA_Sim_InvalidHex]      = "Simulation/Invalid Hex",
    [NMA_Sim_ChecksumErr]     = "Simulation/Checksum Error",
    [NMA_Sim_NoStrobe]        = "Simulation/No Strobe",
    [NMA_Sim_ShortReply]      = "Simulation/Short Reply",
    [NMA_Sim_LongReply]       = "Simulation/Long Reply",
    [NMA_Sim_VOn]             = "Simulation/Voltage On",
    [NMA_Sim_VOff]            = "Simulation/Voltage Off",
    [NMA_Sim_ILOn]            = "Simulation/Local Current On",
    [NMA_Sim_ILOff]           = "Simulation/Local Current Off",
    [NMA_Sim_IROn]            = "Simulation/Remote Current On",
    [NMA_Sim_IROff]           = "Simulation/Remote Current Off",
    [NMA_Sim_ClockOffset]     = "Simulation/Clock Offset",
#endif
  };
}  // namespace PPI

#endif
