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
@brief SOML Casting Furnace Power Panel Interface firmware
@author Michael Sibayan
@date January 27, 2021
@file main.cpp

This file contains the firmware code for the casting furnace's power panel
interface. This interface acts as an MQTT translator to the existing power
controllers.
*/

#include "ppi_global.hpp"
#include "ppi.hpp"
#include "ntp_thread.hpp"

#include "TeensyThreads.h"


PowerPanelInterface ppi; // Power Panel Interface networking

// Ensure single-threaded access to the network
Threads::Mutex network_lock;

// NTP Servers
IPAddress ntp_server_ip[NUM_BROKERS] = {
  IPAddress(MQTT_BROKER_1),
  IPAddress(MQTT_BROKER_2)
};

const int ledPin = 13; // diagnositic LED pin number
bool init_okay = false;

typedef enum blink_code_enum{
  BLINK_SOLID = 0,
  BLINK_1 = 1,
  BLINK_2 = 2,
  BLINK_3 = 3,
  BLINK_4 = 4
} blink_code_t;


// ============== Diagnostic LED Thread ===================================== //
volatile blink_code_t blinkcode = BLINK_SOLID;
void ledThread() {
  pinMode(ledPin, OUTPUT);
  while(1) {
    if(blinkcode == BLINK_SOLID){
      digitalWrite(ledPin, HIGH);
    }
    else{
      blink_code_t blinkcode_copy = blinkcode;
      for(int i = 0; i < blinkcode_copy; i++){
        digitalWrite(ledPin, LOW);
        threads.delay(200);
        digitalWrite(ledPin, HIGH);
        threads.delay(200);
      }
      digitalWrite(ledPin, LOW);
    }
    threads.delay(2000);
  }
}


// ============== Setup all objects ========================================= //
void setup(){
  pinMode(ledPin, OUTPUT);

  // Config ID jumper GPIO
  pinMode(JUMPER_PIN_1, INPUT_PULLUP);
  pinMode(JUMPER_PIN_2, INPUT_PULLUP);
  pinMode(JUMPER_PIN_3, INPUT_PULLUP);
  pinMode(JUMPER_PIN_4, INPUT_PULLUP);
  pinMode(JUMPER_PIN_5, INPUT_PULLUP);

  #ifdef DEBUG
  //Serial.begin(9600);
  Serial.begin(115200);
  // Wait for the serial port to connect.  Needed for native USB port only.
  // Stop waiting if it hasn't connected after 5 seconds.
  for(int i = 0; i < 50 && !Serial; i++){
    // Flash the LED to let the user know we're waiting
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
  }
  #endif

  // Report our version
  DebugPrint(PPI_VERSION_COMPLETE);

  digitalWrite(ledPin, LOW);

  blinkcode = BLINK_SOLID;

  threads.setSliceMicros(500);
  threads.addThread(ledThread);

  // Select which serial port to use based on the Jumper 5 setting
  int main_port = PPI_JP5 ? 2 : 1;
  DebugPrintNoEOL("Main port = Serial");
  DebugPrint(main_port);

  // Test MetricNames[], mainly just to avoid a compiler warning
  if(PPI::MetricNames[0] == NULL)
    DebugPrint("No metric names defined?");

  // Start the power panel on the serial port
  ppi.start_panel(main_port);

  // configure network, device id, mqtt
  init_okay = ppi.network_init(&network_lock);
  if(init_okay){
    // Pause several seconds to allow network code to stabilize
    threads.delay(4000);

    // Start the NTP update thread
    ntp_start(ntp_server_ip, NUM_ELEM(ntp_server_ip), &network_lock);
  }
}


// =============== Main Loop ================================================ //
void loop(){
  if(init_okay){
    ppi.update_status(false);

    if(!ppi.check_broker())
      // Not connected to a broker
      blinkcode = BLINK_1;
    else if(!ppi.is_primary_host_online())
      // Connected to a broker but Primary Host is offline
      blinkcode = BLINK_2;
    else
      // Primary Host is online
      blinkcode = BLINK_3;
  }

  threads.delay(5);
}
