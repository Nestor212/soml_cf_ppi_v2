/*******************************************************************************
Copyright 2021
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
@brief Global settings for the power panel interfaces
@date January 27, 2021
@file ppi_global.hpp

This file specifies global settings for the PPI modules.
*/

#ifndef __PPI_GLOBAL_H__
#define __PPI_GLOBAL_H__


// Overall version of the PPI module
#define PPI_VERSION  "1.5"

// Set the active components of the overall PPI module version string
#ifdef DEVELOPMENT_SYSTEM
  #define SYSTEM_VERSION ", DEV NET"
#else
  #define SYSTEM_VERSION ""
#endif

#ifdef DEBUG
  #define DEBUG_VERSION ", DEBUG"
#else
  #define DEBUG_VERSION ""
#endif

#ifdef SIMULATED_POWER_PANEL
  #define SIMULATED_POWER_PANEL_VERSION ", POWER PANEL SIMULATED"
#else
  #define SIMULATED_POWER_PANEL_VERSION ""
#endif

#ifdef SIMULATED_SERIAL_PORTS
  #define SIMULATED_SERIAL_PORTS_VERSION ", SERIAL PORTS SIMULATED"
#else
  #define SIMULATED_SERIAL_PORTS_VERSION ""
#endif

#ifdef LINUX_SIMULATOR
  #define LINUX_SIMULATOR_VERSION ", LINUX"
#else
  #define LINUX_SIMULATOR_VERSION ""
#endif

// PPI module version string including optional features
#define PPI_VERSION_COMPLETE  PPI_VERSION \
                              SYSTEM_VERSION \
                              DEBUG_VERSION \
                              SIMULATED_POWER_PANEL_VERSION \
                              SIMULATED_SERIAL_PORTS_VERSION \
                              LINUX_SIMULATOR_VERSION

// Display diagnostic messages on USB serial port if debugging is enabled
#ifdef DEBUG
#define DebugPrint(msg)       Serial.println(msg)
#define DebugPrintNoEOL(msg)  Serial.print(msg)
#else
#define DebugPrint(msg)
#define DebugPrintNoEOL(msg)
#endif

#endif  // __PPI_GLOBAL_H__
