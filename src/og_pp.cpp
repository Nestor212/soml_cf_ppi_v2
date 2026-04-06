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
@date Janary 27, 2021
@file pp.cpp

Implementation of the control interface to a single power panel
*/

#include <Arduino.h>
#include "TeensyThreads.h"
#include <cstring>
#include <math.h>
#include "pp.hpp"
#include "ntp_thread.hpp"


#ifdef SIMULATED_SERIAL_PORTS
  #include "pp_sim.hpp"
  #define SERIAL_PORT_1 SimSerial1
  #define SERIAL_PORT_2 SimSerial2
#else
  #define SERIAL_PORT_1 Serial1
  #define SERIAL_PORT_2 Serial2
#endif


#define STROBE_CHAR   0x0A       //!< @brief Marks the end of a serial message

#define PP_CYCLE_PERIOD_MILLIS    250  //!< @brief Nominal time between each sent strobe (ms)
#define PP_CYCLE_PERIOD_MICROS  (PP_CYCLE_PERIOD_MILLIS * 1000)
                                       //!< @brief Nominal cycle period in microseconds

#define MAX_START_OFFSET_MS  60000     // Maximum start time delta for a new list (ms)

// Cycle offset correction settings
#define MAX_OFFSET_MS         1200     // Maximum cycle offset that we try to correct (ms)
#define MIN_OFFSET_MS            5     // Minimum cycle offset that we try to correct (ms)
#define MAX_CYCLE_ADJ_MS         5     // Maximum adjustment to nominal cycle time (ms)
#define MEAS_OFFSET_CYCLES  (10 * 4)   // Over how many cycles we measure the offset


// Helper functions

/**
  @brief Return the absolute value of a 64-bit signed integer.

  This function is here to avoid potential truncation on 32-bit architectures.

  @param value a 64-bit signed integer
  @return the absolute value of the input parameter
*/
int64_t abs64(int64_t value){
  if(value >= 0)
    return value;
  return -value;
}


/**
  @brief Convert a single 7-bit hex character to an unsigned integer
  @param asciiHex a 7-bit hex character
  @param output memory location to store the data
  @return false on error, true on success
*/
bool hexCharToInt(uint8_t asciiHex, uint8_t* output){
  uint8_t byte_in = asciiHex & 0x7F;
  if(byte_in >= '0' && byte_in <= '9')
    *output = byte_in - '0';
  else if(byte_in >= 'A' && byte_in <= 'F')
    *output = byte_in - 'A' + 10;
  else if(byte_in >= 'a' && byte_in <= 'f')
    *output = byte_in - 'a' + 10;
  else
    // Not a hex character
    return false;

  // Success
  return true;
}


/**
  @brief Convert a string containing an 8-bit hex value to an unsigned integer
  @param asciiHex string containing 2 characters
  @param output memory location to store the data
  @return false on error, true on success
*/
bool hexStrToInt(uint8_t* asciiHex, uint8_t* output){
  uint8_t hi, lo;

  if(!hexCharToInt(asciiHex[0], &hi))
    return false;
  if(!hexCharToInt(asciiHex[1], &lo))
    return false;

  *output = (hi<<4) | lo;
  return true;
}

/**
  @brief Check that a sequence of bytes has even parity
  @param data string containing num_bytes characters
  @param num_bytes number of bytes in the string
  @return false on error, true on success
*/
bool check_even_parity(uint8_t *data, int num_bytes){
  int i;
  for(i = 0; i < num_bytes; i++){
    uint8_t byte_in = data[i];
    int num_ones = 0;
    int bit;
    for(bit = 0; bit < 8; bit++)
      if(byte_in & (1 << bit))
        num_ones++;
    if(num_ones % 2 != 0)
      // Wrong parity
      return false;
  }

  // All bytes have even parity
  return true;
}



// =============== Power Panel Thread ======================================= //
// Pointer to the power panel used by the thread function
PowerPanel *g_pp = NULL;

void ppThread(void){
  while(1){
    if(g_pp != NULL)
      g_pp->tick();
    threads.yield();
  }
}


// Default constructor.
PowerPanel::PowerPanel(){
  this->serial = NULL;
  this->switched_list = false;
  this->active_modified = false;
  this->state_machine = PP_STATEMACHINE_START;
  memset(&this->data, 0, sizeof(this->data));
  memset(&this->this_cycle, 0, sizeof(this->this_cycle));
  memset(&this->last_cycle, 0, sizeof(this->last_cycle));
  this->this_cycle.command_index = -1;
  this->last_cycle.command_index = -1;
  this->lamps = 0;

  startOffsetSequence(0.0);
  this->avg_cycle_offset_us = 0;
  this->ideal_cycle_ntp_ms = 0;
  this->cycle_ntp_ms = 0;
  this->cycle_time_us = 0;
  this->next_cycle_us = 0;

  memset(this->tx_data, 0, sizeof(this->tx_data));
  memset(this->rx_buffer, 0, sizeof(this->rx_buffer));
  this->rx_bytes = 0;
}


// Set initial values and port number, start serial port, start state machine.
bool PowerPanel::start(unsigned int port){
  // Select serial port to use
  if(port == 1)
    this->serial = &SERIAL_PORT_1;
  else if(port == 2)
    this->serial = &SERIAL_PORT_2;
  else{
    // Invalid port number
    this->serial = NULL;
    return false;
  }

#ifdef SIMULATED_SERIAL_PORTS
  // Set up a simulated loop-back cable between the simulated serial ports by
  // connecting the output of each simulated port to the other port's input
  SimSerial1.set_output(&SimSerial2);
  SimSerial2.set_output(&SimSerial1);
#endif

  memset(&this->data, 0, sizeof(this->data));
  memset(this->tx_data, 0, sizeof(this->tx_data));

  this->serial->begin(PP_SERIAL_BAUD, PP_SERIAL_FORMAT);

  // Start transmitting
  this->state_machine = PP_STATEMACHINE_START;

  // Start the thread to run the power panel
  g_pp = this;
  threads.addThread(ppThread);

  return true;
}


// Return feedback data from this object.
void PowerPanel::getCurrentData(power_panel_data_t* ptr){
  this->data_lock.lock(); // <---------- DATA MUTEX LOCK
  *ptr = this->data;
  this->data_lock.unlock(); // <---------- DATA MUTEX UNLOCK
}


// Convert a power panel status code to a human-readable string.
const char * PowerPanel::panelStatusToString(pp_status_t panel_status){
  switch(panel_status){
    case PP_STATUS_IDLE:            return "Idle";
    case PP_STATUS_ACTIVE:          return "Active";
    case PP_STATUS_TIMEOUT:         return "Reply timeout";
    case PP_STATUS_PARITY_ERROR:    return "Parity error";
    case PP_STATUS_INVALID_HEXCHAR: return "Invalid hex character";
    case PP_STATUS_CHECKSUM_ERROR:  return "Checksum error";
    case PP_STATUS_MISSING_STROBE:  return "Missing strobe";
    case PP_STATUS_UNDERRUN:        return "Reply too short";
    case PP_STATUS_OVERRUN:         return "Reply too long";
    default:                        return "Unknown";
  }
}


// Set the data to be sent to the error lamp output channels.
void PowerPanel::setErrorLampData(uint32_t data){
  this->lamps = data;
}


// Return the data currently being sent to the error lamp output channels.
uint32_t PowerPanel::getErrorLampData(){
  return this->lamps;
}


// Accept a new list for processing as the "next" list.
bool PowerPanel::writeNextList(uint32_t *list, uint32_t size,
                               bool apply_start, uint64_t start_ntp_ms){
  list_lock.lock(); // <---------- LIST MUTEX LOCK

  this->received_pending_list = this->pending_list.setList(list, size);
  if(this->received_pending_list){
    this->received_pending_start = apply_start;
    this->pending_start_ntp_ms   = start_ntp_ms;
  }
  else
    this->received_pending_start = false;

  list_lock.unlock(); // <---------- LIST MUTEX UNLOCK

  // Return true if the new list was accepted
  return this->received_pending_list;
}


// The main power panel loop.  This is the main processing function for a Power
// Panel and it should be executed regularly.  Timing is handled internally.
void PowerPanel::tick(){
  if(this->serial == NULL)
    // Nothing to do if no serial port set up
    return;

  switch(this->state_machine){
    case PP_STATEMACHINE_START:
      // Send the first command message
      this->setTxData();
      this->writeSerial();

      // Start the timer
      this->next_cycle_us = micros();
      this->state_machine = PP_STATEMACHINE_STROBE;
      // Drop-through to the next case for quicker response

    case PP_STATEMACHINE_STROBE:
      // Send the strobe character marking the start of a timing cycle
      this->writeStrobe();

      // Send the next command message (without the trailing strobe character)
      this->setTxData();
      this->writeSerial();

      // Now wait for an incoming response
      this->rx_bytes = 0;
      this->state_machine = PP_STATEMACHINE_WAIT;
      // Drop-through to the next case for quicker response

    case PP_STATEMACHINE_WAIT:
    {
      // Look for any response while waiting for the start of the next timing
      // cycle
      int byte_in = -1; // last byte received, -1 if no data available
      if(this->serial != NULL)
        byte_in = this->serial->read();

      // Did we receive a character?
      if(byte_in != -1){
        // Append the new character to the end of the current receive buffer
        if(this->rx_bytes >= PP_RX_NUM_DATA_SYMBOLS){
          // Message is too long - discard the first character
          this->rx_bytes = PP_RX_NUM_DATA_SYMBOLS - 1;
          memmove(this->rx_buffer, this->rx_buffer + 1, this->rx_bytes);
        }
        this->rx_buffer[this->rx_bytes] = byte_in;
        this->rx_bytes++;

        // Did we receive the strobe character marking the end of the message?
        if(byte_in == STROBE_CHAR){
          if(this->rx_bytes == PP_RX_NUM_DATA_SYMBOLS)
            parseResponse();
          else
            setErrorStatus(PP_STATUS_UNDERRUN);
        }
      }

      // Have we reached the next timing cycle?
      // Note that this works even when the times wraparound back to zero
      int32_t dt = micros() - this->next_cycle_us;
      if(dt >= 0){
        // Report an error if we didn't receive a complete response from the
        // power panel
        if(this->rx_bytes == 0)
          setErrorStatus(PP_STATUS_TIMEOUT);
        else if(this->rx_buffer[this->rx_bytes-1] != STROBE_CHAR)
        {
          if(this->rx_bytes < PP_RX_NUM_DATA_SYMBOLS)
            setErrorStatus(PP_STATUS_TIMEOUT);
          else
            setErrorStatus(PP_STATUS_MISSING_STROBE);
        }

        // Start the next cycle
        this->state_machine = PP_STATEMACHINE_STROBE;
      }
      break;
    }

    default:
      this->state_machine = PP_STATEMACHINE_START;
  }
}


// Send the strobe character to the power panel, marking the end of a complete
// transmission and the start of the timing sequence.  Also update cycle data
// for the next cycle.
void PowerPanel::writeStrobe(){
  if(this->serial != NULL)
    this->serial->write(STROBE_CHAR);

  // Record the time when the strobe character was sent
  // Note: these functions are called explicitly in this order to minimize
  // latency
  this->cycle_time_us = micros();                      // Read microsecond clock first
  this->cycle_ntp_ms  = ntp_get_current_time_millis(); // Read millisecond NTP clock second

  // Yield the thread time-slice after each write to prevent locking out the
  // other threads during long transmissions
  threads.yield();

  // Update the data that has been completely sent.  The cycle timing is such
  // that a response received from the power panel matches the command sent
  // prior to the strobe, not any new command sent after the strobe.  This also
  // applies to the lists if they were switched or modified since the last
  // strobe was sent.
  memcpy(&this->last_cycle, &this->this_cycle, sizeof(this->last_cycle));
  this->switched_list = false;
  this->active_modified = false;

  // Calculate the next cycle period
  calculateCyclePeriod();

  // Advance the cycle timer
  // Note that this works even when the times wraparound back to zero
  this->next_cycle_us += this->cycle_period_us;
}


// Calculate the period of the next cycle to correct any offset in the current
// cycle time from NTP.
void PowerPanel::calculateCyclePeriod(void){
  // Calculate how far this cycle is offset from its desired time.  Note that
  // this calculation works even for large differences or when they wraparound
  // to zero.
  int64_t cycle_offset_ms = (int64_t) (cycle_ntp_ms - ideal_cycle_ntp_ms);

  // Update the strobe count
  strobe_count++;

  // Calculate the NTP time when the next strobe character should be sent
  ideal_cycle_ntp_ms += PP_CYCLE_PERIOD_MILLIS;

  // Check the magnitude of this cycle's offset
  if(abs64(cycle_offset_ms) > MAX_OFFSET_MS){
    // There's been a sudden change in the offset - reset the ideal cycle times
    resetCycleTime(cycle_offset_ms);
    return;
  }

  // Calculate the cumulative cycle time offset
  sum_cycle_offset_ms += cycle_offset_ms;

  if(strobe_count < sequence_length)
    // Haven't finished the current measurement or correction sequence yet
    return;

  if(correcting_offset){
    // We've finished the current correction sequence - stop correcting and
    // start a measurement sequence
    startOffsetSequence(0.0);
    return;
  }

  // We've finished the current measurement sequence
  // Calculate the average cycle offset
  double avg_offset_ms = sum_cycle_offset_ms / strobe_count;
  avg_cycle_offset_us = round(avg_offset_ms * 1000.0);

  // Act based on the magnitude of the offset
  double abs_offset_ms = fabs(avg_offset_ms);
  if(abs_offset_ms > MAX_OFFSET_MS){
    // For large offsets just reset the ideal cycle time to NTP
    resetCycleTime(round(avg_offset_ms));
    return;
  }
  else if(abs_offset_ms >= MIN_OFFSET_MS)
    // The offset is moderate - adjust the cycle time to correct it
    startOffsetSequence(avg_offset_ms);
  else
    // Ignore small offsets - start a new offset measurement sequence
    startOffsetSequence(0.0);
}


// Adjust the ideal cycle time by the specified offset to reset it to NTP, then
// start a new measurement sequence.
void PowerPanel::resetCycleTime(int64_t offset_ms){
  // Adjust the ideal cycle time
  ideal_cycle_ntp_ms += offset_ms;

  // Recalculate the start time for the active list, if it's been set
  if(this_cycle.active_start_ntp_ms != 0)
    this_cycle.active_start_ntp_ms += offset_ms;
  if(last_cycle.active_start_ntp_ms != 0)
    last_cycle.active_start_ntp_ms += offset_ms;

  // Zero the cycle offset
  avg_cycle_offset_us = 0;

  // Start a new offset measurement sequence
  startOffsetSequence(0.0);

  DebugPrintNoEOL("Resetting large cycle offset (ms as 32MSb,32LSb): ");
  // Display the 64-bit integer as two 32-bit integers
  DebugPrintNoEOL(offset_ms >> 32);
  DebugPrintNoEOL(",");
  DebugPrint(offset_ms & 0xFFFFFFFF);
}


// Start a sequence to either measure or correct the cycle offset.
void PowerPanel::startOffsetSequence(double cycle_offset_ms){
  // If cycle offset is non-zero we'll correct it; otherwise we'll measure it
  this->correcting_offset = (cycle_offset_ms != 0.0);

  if(this->correcting_offset){
    // We're correcting an offset - calculate the number of adjustment cycles
    // and the new cycle period
    this->sequence_length = fabs(cycle_offset_ms / MAX_CYCLE_ADJ_MS) + 1;
    int32_t correction_us = round(-cycle_offset_ms * 1000.0 / this->sequence_length);
    this->cycle_period_us = PP_CYCLE_PERIOD_MICROS + correction_us;
  }
  else{
    // We're measuring the offset - use the default period and measurement
    // sequence length
    this->cycle_period_us = PP_CYCLE_PERIOD_MICROS;
    this->sequence_length = MEAS_OFFSET_CYCLES;
  }

  // Reset the counters and cumulative offset to start the new sequence
  this->strobe_count = 0;
  this->sum_cycle_offset_ms = 0.0;
}


// Get an array of bytes to send to the power panel (will get converted to hex).
void PowerPanel::setTxData(){
  // Get the next heater output data
  uint32_t list_data = 0;
  int32_t data_index = -1;
  list_lock.lock(); // <---------- LIST MUTEX LOCK

  // Register the pending list (if any) as the next list
  // Note: This is done here so that it happens at a predictable time in the cycle
  processPendingList();

  if(!this->active_list.getNextElement(&list_data, &data_index)){
    // The active list is empty - load the next list into it
    this->old_active_list.moveList(&this->active_list);
    this->active_list.moveList(&this->next_list);
    this->active_list.getNextElement(&list_data, &data_index);
    this->switched_list = true;

    // Calculate the start time for the new active list if it's not empty as well
    // as any list that might follow it
    if(data_index != -1){
      // This is the ideal NTP time for the next strobe, minus any elements
      // that have been skipped
      this->this_cycle.active_start_ntp_ms = this->ideal_cycle_ntp_ms -
                                               data_index * PP_CYCLE_PERIOD_MILLIS;

      // The next list immediately follows the active list
      this->this_cycle.next_start_ntp_ms = this->ideal_cycle_ntp_ms +
                                             (this->active_list.elementsLeft() + 1) *
                                             PP_CYCLE_PERIOD_MILLIS;
    }
  }
  list_lock.unlock(); // <---------- LIST MUTEX UNLOCK

  this->this_cycle.command_output = list_data;
  this->this_cycle.command_index  = data_index;

  memset(tx_data, 0, sizeof(tx_data));

  // separate heater output data into bytes
  this->tx_data[0] = (list_data      ) & 0xFF;
  this->tx_data[1] = (list_data >>  8) & 0x7F;
  this->tx_data[2] = (list_data >> 15) & 0xFF;
  this->tx_data[3] = (list_data >> 23) & 0x7F;

  // separate error lamp outputs into bytes
  this->tx_data[4] = (this->lamps      ) & 0xFF;
  this->tx_data[5] = (this->lamps >>  8) & 0x7F;
  this->tx_data[6] = (this->lamps >> 15) & 0xFF;
  this->tx_data[7] = (this->lamps >> 23) & 0x7F;

  // this->tx_data[8] to this->tx_data[15] are zero

  // compute checksum
  uint16_t checksum = 0;
  int i;
  for(i=0; i<8; i++)
    checksum += tx_data[i];

  this->tx_data[16] = checksum & 0xFF;
}


// Fill the next list with data from the pending list.
void PowerPanel::processPendingList(){
  if(!this->received_pending_list)
    // No pending list - nothing to do
    return;

  // Move the pending list into the next list
  this->next_list.moveList(&this->pending_list);

  // Calculate the nominal start time for the next list as the ideal NTP time
  // for the next strobe plus any elements remaining in the active list
  this->this_cycle.next_start_ntp_ms = this->ideal_cycle_ntp_ms +
                                         this->active_list.elementsLeft() *
                                         PP_CYCLE_PERIOD_MILLIS;

  // Adjust the start time of the next list if specified
  if(this->received_pending_start){
    // Calculate the start time offset
    int32_t offset_ms = this->pending_start_ntp_ms - this->this_cycle.next_start_ntp_ms;

    // Only accept the new start time if it falls in the valid range
    if(offset_ms >= -MAX_START_OFFSET_MS &&
       offset_ms <=  MAX_START_OFFSET_MS){
      // Record the new start time
      this->this_cycle.next_start_ntp_ms = this->pending_start_ntp_ms;

      // Convert offset to whole ticks and fractional ticks (in milliseconds)
      int32_t ticks = offset_ms / PP_CYCLE_PERIOD_MILLIS;
      int32_t subtick_ms = offset_ms % PP_CYCLE_PERIOD_MILLIS;
      // Adjust so fractional ticks are within a half-tick of zero
      if(subtick_ms > PP_CYCLE_PERIOD_MILLIS / 2){
        subtick_ms -= PP_CYCLE_PERIOD_MILLIS;
        ticks++;
      }
      else if(subtick_ms < -PP_CYCLE_PERIOD_MILLIS / 2){
        subtick_ms += PP_CYCLE_PERIOD_MILLIS;
        ticks--;
      }
      if(subtick_ms != 0){
        // Adjust cycle NTP times to (gradually) apply the fractional offset
        this->ideal_cycle_ntp_ms += subtick_ms;
        // Start a new offset measurement sequence
        startOffsetSequence(0.0);
      }
      if(ticks > 0){
        // Delay the start of the next list by repeating the last part of the
        // active list
        this->active_list.extend(ticks);

        // If the active list was empty, calculate its new start time as the
        // ideal NTP time for the next strobe
        if(this->this_cycle.command_index == -1)
          this->this_cycle.active_start_ntp_ms = this->ideal_cycle_ntp_ms;

        // Delay reporting the modified active list by one cycle
        this->active_modified = true;
      }
      else if(ticks < 0){
        // Advance the start of the next list by truncating the active list
        // and/or skipping the first part of the next list
        ticks *= -1;
        ticks = this->active_list.truncate(ticks);
        if(ticks > 0)
          this->next_list.setIndex(ticks);

        // Delay reporting the modified active list by one cycle
        this->active_modified = true;
      }
    }
  }

  // The pending list has been processed
  this->received_pending_list  = false;
  this->received_pending_start = false;
}


// Send the command message to the power panel, but without the strobe
// character.  This will be sent separately to trigger the start of the
// timing sequence.
void PowerPanel::writeSerial(){
  int i;
  if(this->serial != NULL){
    // Yield the thread time-slice after each write to prevent locking out the
    // other threads during long transmissions
    for(i=0; i<PP_TX_NUM_DATA_BYTES; i++){
      if(this->tx_data[i] <= 0x0F){
        this->serial->print("0");
        threads.yield();
      }
      this->serial->print(this->tx_data[i], HEX);
      threads.yield();
    }
  }
}


// Parse the response and set the object data.
void PowerPanel::parseResponse(){
  int i;
  uint8_t data[PP_RX_NUM_DATA_BYTES];

  // Check the received characters have the correct parity
  if(!check_even_parity(this->rx_buffer, PP_RX_NUM_DATA_SYMBOLS)){
    setErrorStatus(PP_STATUS_PARITY_ERROR);
    return;
  }

  // Convert the hex character string into bytes
  for(i=0; i<PP_RX_NUM_DATA_BYTES; i++){
    if(!hexStrToInt(this->rx_buffer + (2*i), &data[i])){
      setErrorStatus(PP_STATUS_INVALID_HEXCHAR);
      return;
    }
  }

  // compute and compare checksum
  uint8_t checksum = 0; // computed
  for(i=0; i<PP_RX_NUM_DATA_BYTES-1; i++)
    checksum += data[i];

  if(checksum != data[PP_RX_NUM_DATA_BYTES-1]){
    setErrorStatus(PP_STATUS_CHECKSUM_ERROR);
    return;
  }

  // Check the strobe character terminates the message
  if(this->rx_buffer[PP_RX_NUM_DATA_SYMBOLS-1] != STROBE_CHAR){
    // Note that this should never trigger
    setErrorStatus(PP_STATUS_OVERRUN);
    return;
  }

  uint32_t v_mon  = data[0] + ((data[1] & 0x7F) << 8) + (data[2]  << 15) + ((data[3]  & 0x7F) << 23);
  uint32_t il_mon = data[4] + ((data[5] & 0x7F) << 8) + (data[6]  << 15) + ((data[7]  & 0x7F) << 23);
  uint32_t ir_mon = data[8] + ((data[9] & 0x7F) << 8) + (data[10] << 15) + ((data[11] & 0x7F) << 23);

  // Success
  pp_status_t status = PP_STATUS_IDLE;
  if(this->last_cycle.command_index != -1)
    // We're currently sending out commands from the active list
    status = PP_STATUS_ACTIVE;
  setStatus(v_mon, il_mon, ir_mon, status);
}


// Set our status to indicate an error with the power panel.
void PowerPanel::setErrorStatus(pp_status_t panel_status){
  setStatus(0, 0, 0, panel_status);
}


// Update our reported status of the power panel.
void PowerPanel::setStatus(uint32_t v_mon, uint32_t il_mon, uint32_t ir_mon,
                           pp_status_t panel_status){
  this->data_lock.lock();  // <---------- DATA MUTEX LOCK
  this->data.command_counter++;  // This indicates the data has changed
  this->data.panel_status = panel_status;
  // Note that we report the command that was completely sent as of the last
  // strobe, not the one that is queued, as this matches the response from the
  // power panel.  This also applies to the lists if they were switched since
  // the last strobe was sent.
  // By default return the current lists
  PPList *active = &this->active_list;
  PPList *next   = &this->next_list;
  uint64_t next_start_time = this->this_cycle.next_start_ntp_ms;
  if(this->switched_list){
    // The lists were just switched so return the previous lists instead
    active = &this->old_active_list;
    next = &this->active_list;
    next_start_time = this->last_cycle.next_start_ntp_ms;
  }
  // If the active list has just been modified, delay updating it by one cycle
  if(!this->active_modified)
    active->getList(this->data.active_schedule,
                    &this->data.active_schedule_size);
  next->getList(this->data.next_schedule,
                &this->data.next_schedule_size);
  this->data.active_start_time = this->last_cycle.active_start_ntp_ms;
  this->data.next_start_time   = next_start_time;
  this->data.command_index = this->last_cycle.command_index;
  this->data.command_sent_ntp_ms = this->cycle_ntp_ms;
  this->data.command_sent_micros = this->cycle_time_us;
  this->data.heater_output = this->last_cycle.command_output;
  this->data.v_mon  = v_mon;
  this->data.il_mon = il_mon;
  this->data.ir_mon = ir_mon;
  this->data.cycle_period = this->cycle_period_us;
  this->data.cycle_offset = this->avg_cycle_offset_us;
  this->data_lock.unlock();  // <---------- DATA MUTEX UNLOCK

  // Display any errors in the last 10 seconds (40 ticks)
  static int s_counts[ PP_NUM_STATUS + 1 ] = { 0 };
  static int s_counter = 0;
  if(panel_status > PP_NUM_STATUS)
    panel_status = PP_NUM_STATUS;
  s_counts[ panel_status ]++;
  s_counter++;
  if(s_counter >= 40){
    if(s_counts[ PP_STATUS_IDLE ] + s_counts[ PP_STATUS_ACTIVE ] != s_counter){
      // Errors occurred - report them
      DebugPrintNoEOL("PP errors last 10 sec:");
      int i;
      for(i = PP_STATUS_ACTIVE + 1; i < PP_NUM_STATUS + 1; i++)
        if(s_counts[ i ] != 0){
          DebugPrintNoEOL(" ");
          DebugPrintNoEOL(panelStatusToString((pp_status_t) i));
          DebugPrintNoEOL("=");
          DebugPrintNoEOL(s_counts[ i ]);
          s_counts[ i ] = 0;
        }
      DebugPrint("");
    }
    s_counts[ PP_STATUS_IDLE   ] = 0;
    s_counts[ PP_STATUS_ACTIVE ] = 0;
    s_counter = 0;
  }
}


// ----- Power Panel Output List helper class --------------------------------//

// Default constructor.
PPList::PPList(){
  zeroList();
}


// Zero out and remove all elements from the list.
void PPList::zeroList(void){
  memset(this->elements, 0, sizeof(this->elements));
  this->size = 0;
  this->index = 0;
  this->repeat = 0;
}


// Return element and increment list index.
bool PPList::getNextElement(uint32_t* data, int32_t* data_index){
  // Check parameters are valid
  if(data == NULL || data_index == NULL)
    return false;

  // Have we reached the end of the list?
  if(this->index >= this->size){
    // Check whether we want to repeat some elements
    uint32_t num_to_repeat = this->repeat;
    if(num_to_repeat > this->size)
      // Limit the repeated elements to the total list
      num_to_repeat = this->size;

    // Rewind the index to repeat elements, if any
    this->index = this->size - num_to_repeat;
    this->repeat -= num_to_repeat;

    // Are we still at the end of the list?
    if(this->index >= this->size){
      // Set the return values to indicate no more elements
      *data = 0;
      *data_index = -1;
      return false;
    }
  }

  // Return the next element
  *data = elements[this->index];
  *data_index = this->index;

  // Increment the list index
  this->index++;

  // Success
  return true;
}


// Return the number of elements left to run in the list, including any
// repeats.  Returns 0 if the list is empty or has been completely run.
uint32_t PPList::elementsLeft(void){
  if(this->size == 0)
    // The list is empty
    return 0;

  if(this->index >= this->size)
    // We've reached the end of the list, so return any repeats to be run
    return this->repeat;

  // We're somewhere in the middle of the list, so return the remaining
  // elements plus any repeats to be run
  return this->size - this->index + this->repeat;
}


// Return the full list of elements.  Note that the caller_array (if supplied)
// must be large enough to hold all the elements in the list.
void PPList::getList(uint32_t* caller_array, uint32_t* caller_size){
  if(caller_array != NULL)
    memcpy(caller_array, this->elements, this->size * sizeof(*caller_array));

  if(caller_size != NULL)
    *caller_size = this->size;
}


// Populate the list with data.
bool PPList::setList(uint32_t* data, uint32_t size){
  if(size > PPLIST_MAX_ELEMENTS ||
     (data == NULL && size > 0))
    // Too many elements or required data not provided
    return false;

  // Copy in the new list
  this->size = size;
  if(size > 0)
    memcpy(this->elements, data, size * sizeof(*this->elements));

  // Reset the index to the first element and only run the list once
  this->index = 0;
  this->repeat = 0;
  return true;
}


// Move the contents of the source list into this list, clearing out the source
// list.
void PPList::moveList(PPList* source){
  if(source == NULL)
    // Invalid pointer
    return;

  // Copy the source list, including its current index and any repeats
  setList(source->elements, source->size);
  this->index  = source->index;
  this->repeat = source->repeat;

  // Empty the source list
  source->setList(NULL, 0);
}


// Change the current index.  Setting the index to size or larger will
// effectively empty the list.
void PPList::setIndex(uint32_t new_index){
  this->index = new_index;
}


// Extend the list by the specified number of elements by repeating the last
// part of the list.
void PPList::extend(int32_t add_elements){
  if(add_elements <= 0)
    // Nothing to do
    return;

  // Constrain the number of new elements to the maximum size of the list
  if(add_elements > PPLIST_MAX_ELEMENTS)
    add_elements = PPLIST_MAX_ELEMENTS;

  // If the list is empty just fill it with the required number of zero
  // elements
  if(this->size == 0){
    zeroList();
    this->size = add_elements;
    return;
  }

  // Run the last points of the list twice to "extend" the list.  Note that
  // points will be repeated multiple times if the list is smaller than the
  // number of points to be added.
  this->repeat += add_elements;

  // Constrain the number of repeated elements to the maximum size of the list
  if(this->repeat > PPLIST_MAX_ELEMENTS)
    this->repeat = PPLIST_MAX_ELEMENTS;
}


// Truncate the list by the specified number of elements by reducing any repeats
// and/or removing unused elements from the end of the list.  Returns the number
// of elements still remaining to be removed.
int32_t PPList::truncate(int32_t remove_elements){
  if(remove_elements <= 0)
    // Nothing to do
    return 0;

  // Constrain the number of removed elements to the maximum size of the list
  if(remove_elements > PPLIST_MAX_ELEMENTS)
    remove_elements = PPLIST_MAX_ELEMENTS;

  // If the list is empty we can't remove any elements
  if(this->size == 0)
    return remove_elements;

  // First take elements off any to be repeated
  uint32_t repeats_to_remove = remove_elements;
  if(repeats_to_remove > this->repeat)
    repeats_to_remove = this->repeat;
  this->repeat    -= repeats_to_remove;
  remove_elements -= repeats_to_remove;

  // Now skip any unused elements at the end of the list
  int32_t unused_elements = this->size - this->index;
  if(unused_elements < 0)
    unused_elements = 0;
  if(unused_elements > remove_elements)
    unused_elements = remove_elements;
  this->size      -= unused_elements;
  remove_elements -= unused_elements;

  // Return the number of elements still remaining to be removed
  return remove_elements;
}
