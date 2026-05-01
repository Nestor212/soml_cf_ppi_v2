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
@file ppi.cpp

This file contains the firmware code for the casting furnace's power panel
interface. This interface acts as an MQTT translator to the existing power
controller.
*/

#include <cstring>
#include <string.h>
#include <ArduinoJson.h> // STATE messages are JSON encoded
#include "og_ppi.hpp"

// Sparkplug settings
#define GROUP_ID          PPI_GROUP_ID      // This node's group ID, used by NODE_TOPIC()
#define NODE_ID_TEMPLATE  PPI_NODE_ID "x"   // Template for this node's node ID
#define NODE_ID_TOKEN     'x'               // Character to be replaced with module ID

#define NUM_MAC_OCTETS    6                 // Number of octets in a MAC address

StaticJsonDocument<MAX_HOST_STATE_MSG_LEN> json;

namespace {

constexpr uint32_t HEATER_MASK_30 = 0x3FFFFFFFu;

uint32_t relayMaskFromState_(const PowerPanelState& state) {
  uint32_t mask = 0;
  for (uint8_t index = 0; index < HEATER_COUNT; ++index) {
    if (state.heaters[index].relayCommand) {
      mask |= (1UL << index);
    }
  }
  return mask;
}

uint32_t voltageMaskFromState_(const PowerPanelState& state) {
  uint32_t mask = 0;
  for (uint8_t index = 0; index < HEATER_COUNT; ++index) {
    if (state.heaters[index].localVoltage) {
      mask |= (1UL << index);
    }
  }
  return mask;
}

uint32_t localCurrentMaskFromState_(const PowerPanelState& state) {
  uint32_t mask = 0;
  for (uint8_t index = 0; index < HEATER_COUNT; ++index) {
    if (state.heaters[index].localCurrent) {
      mask |= (1UL << index);
    }
  }
  return mask;
}

uint32_t remoteCurrentMaskFromState_(const PowerPanelState& state) {
  uint32_t mask = 0;
  for (uint8_t index = 0; index < HEATER_COUNT; ++index) {
    if (state.heaters[index].remoteCurrent) {
      mask |= (1UL << index);
    }
  }
  return mask;
}

ppi_panel_status_t panelStatusFromState_(const PowerPanelState& state, uint32_t heaterOutput) {
  if (state.localIoValid == SignalValidity::INVALID ||
      state.remoteIoValid == SignalValidity::INVALID) {
    return PPI_PANEL_STATUS_IO_INVALID;
  }
  if (heaterOutput != 0) {
    return PPI_PANEL_STATUS_ACTIVE;
  }
  return PPI_PANEL_STATUS_IDLE;
}

}

// Teensy MAC address
#if defined(LINUX_SIMULATOR)
extern uint32_t HW_OCOTP_MAC1;
extern uint32_t HW_OCOTP_MAC0;
#endif

void teensyMAC(byte mac[NUM_MAC_OCTETS]){
  uint32_t m1 = HW_OCOTP_MAC1;
  uint32_t m2 = HW_OCOTP_MAC0;
  mac[0] = m1 >> 8;
  mac[1] = m1 >> 0;
  mac[2] = m2 >> 24;
  mac[3] = m2 >> 16;
  mac[4] = m2 >> 8;
  mac[5] = m2 >> 0;
}

// Pass an incoming message to the object.
PowerPanelInterface* callback_object;
void callback(char* topic, byte* payload, unsigned int l){
  callback_object->callback_worker(topic, payload, l);
}

// Constructor
PowerPanelInterface::PowerPanelInterface(){
  callback_object = this;
  this->panel = NULL;
  memset(&this->dev, 0, sizeof(this->dev));

  // By default no sensors are down
  this->dev.v_down  = 0x0;
  this->dev.il_down = 0x0;
  this->dev.ir_down = 0x0;

  this->network_lock = NULL;
  this->broker_index = 0;
  this->last_broker_index = 0;
  this->broker_retry_timeout_start = 0;
  this->was_connected = true;   // To skip the first broker retry timeout

  // The initial value of each metric
  // Reset the sequence number so it starts at zero when incremented
  this->bdSeq = (uint8_t) -1;
  this->nodeReboot      = false;
  this->nodeRebirth     = false;
  this->nodeNextServer  = false;
  this->commsVersion    = PPI_COMMS_VERSION;
  this->firmwareVersion = PPI_VERSION_COMPLETE;
  this->uniqueId        = 0;
  this->commsChannel    = 1;
  this->ntpStatus = {
    .synchronized     = false,
    .last_update_time = 0,
    .server_ip        = NULL,
  };
  snprintf(this->ntpServer, sizeof(this->ntpServer), "Not set");
  this->ntpServerPtr    = this->ntpServer;
  this->panelStatus     = "Not set";
  memset(&this->activeSchedule, 0, sizeof(this->activeSchedule));
  memset(&this->nextSchedule,   0, sizeof(this->nextSchedule));
#if defined(SIMULATED_POWER_PANEL)
  memset(&this->sim, 0, sizeof(this->sim));
#endif

  memset(&buffer, 0, sizeof(buffer));
  this->publish_requested = false;

  // configure sparkplug object
  so_sp_initialize(&sp);
  sp.max_metrics = MAX_SPARKPLUG_METRICS;
  sp.metrics = pub_metrics;
  sp.buffer = buffer;
  sp.buffer_len = sizeof(buffer);
  sp.gettimestamp = &ntp_get_current_time_millis;

  // Primary Host tracking
  this->first_broker_connection = true;
  this->primary_host_online     = false;
  this->primary_host_statetime     = 0;
  this->primary_host_timeout_start = 0;
  this->primary_host_timeout       = 0;
}

void PowerPanelInterface::attach_panel(PowerPanel *panelBackend){
  this->panel = panelBackend;
}

// Destructor
PowerPanelInterface::~PowerPanelInterface(){
  // Free up any memory allocated for the schedule datasets
  free_schedule(&this->activeSchedule);
  free_schedule(&this->nextSchedule);
}

// Start up the power panel thread/s.
bool PowerPanelInterface::start_panel(int main_port){
  this->commsChannel = main_port;

  if(this->panel == NULL){
    DebugPrint("No PowerPanel backend attached");
    return false;
  }

#if defined(SIMULATED_POWER_PANEL)
  // Start up the power panel simulator thread on the other serial port first
  if(!this->ppsim.start(3 - main_port))
    DebugPrint("Failed to start Simulated Power Panel");
#endif

  return true;
}

const char *PowerPanelInterface::panelStatusToString_(ppi_panel_status_t status){
  switch(status){
    case PPI_PANEL_STATUS_IDLE:
      return "Idle";
    case PPI_PANEL_STATUS_ACTIVE:
      return "Active";
    case PPI_PANEL_STATUS_IO_INVALID:
      return "I/O Invalid";
    default:
      return "Unknown";
  }
}

void PowerPanelInterface::snapshot_panel_state_(void){
  if(this->panel == NULL){
    return;
  }

  const PowerPanelState& state = this->panel->state();
  PowerPanelScheduleSnapshot sched;
  this->panel->getScheduleSnapshot(sched);
  ppi_data_t& snapshot = this->dev.current_data;

  const uint32_t heaterOutput = relayMaskFromState_(state) & HEATER_MASK_30;
  const uint32_t voltageMask = voltageMaskFromState_(state) & HEATER_MASK_30;
  const uint32_t localCurrentMask = localCurrentMaskFromState_(state) & HEATER_MASK_30;
  const uint32_t remoteCurrentMask = remoteCurrentMaskFromState_(state) & HEATER_MASK_30;

  snapshot.heater_output = heaterOutput;
  snapshot.v_mon = voltageMask;
  snapshot.il_mon = localCurrentMask;
  snapshot.ir_mon = remoteCurrentMask;
  snapshot.panel_status = panelStatusFromState_(state, heaterOutput);
  snapshot.command_counter = sched.commandCounter;
  snapshot.command_index = sched.commandIndex;
  snapshot.command_sent_ntp_ms = sched.commandSentNtpMs;
  snapshot.command_sent_micros = sched.commandSentMicros;
  snapshot.active_start_time = sched.activeStartNtpMs;
  snapshot.next_start_time = sched.nextStartNtpMs;
  snapshot.cycle_period = sched.cyclePeriodUs;
  snapshot.cycle_offset = sched.cycleOffsetUs;

  snapshot.active_schedule_size = sched.activeScheduleSize;
  for(uint16_t idx = 0; idx < sched.activeScheduleSize; ++idx){
    snapshot.active_schedule[idx] = sched.activeSchedule[idx];
  }
  snapshot.next_schedule_size = sched.nextScheduleSize;
  for(uint16_t idx = 0; idx < sched.nextScheduleSize; ++idx){
    snapshot.next_schedule[idx] = sched.nextSchedule[idx];
  }
}

bool PowerPanelInterface::apply_next_schedule_(const uint32_t* elements,
                                               uint32_t num_elements,
                                               bool scheduleStartSet,
                                               uint64_t scheduleStart){
  if(num_elements > PPLIST_MAX_ELEMENTS){
    return false;
  }

  uint32_t masked[PPLIST_MAX_ELEMENTS] = {0};
  for(uint32_t index = 0; index < num_elements; ++index){
    masked[index] = heaters_only(elements[index]);
  }

  if(this->panel == NULL){
    return false;
  }
  return this->panel->writeNextList(masked,
                                    static_cast<uint16_t>(num_elements),
                                    scheduleStartSet,
                                    scheduleStart);
}

/** Perform network start-up configuration
  1) set up the metric arrays
  2) determine where we are installed, based on jumpers
  3) do network configurations (IP address is jumper dependent)
  4) set node, device, and topic names
  Returns false if an error occurs; otherwise returns true.
 **/
bool PowerPanelInterface::network_init(Threads::Mutex *net_lock){
  // Set up the Active and Next Schedule dataset metrics for holding the power
  // commands
  if(!set_up_schedule_dataset(&this->activeSchedule, PPLIST_MAX_ELEMENTS)){
    DebugPrint("Failed to set up Active Schedule dataset");
    return false;
  }
  if(!set_up_schedule_dataset(&this->nextSchedule, PPLIST_MAX_ELEMENTS)){
    DebugPrint("Failed to set up Next Schedule dataset");
    return false;
  }

  // All node metrics except bdseq
  static metric_spec_t s_NodeMetrics[NUM_NODE_METRICS] = {
  //{name, alias,                writable, datatype,                 variable,                                  updated, timestamp}
    {NULL, NMA_Reboot,              true,  METRIC_DATA_TYPE_BOOLEAN, &this->nodeReboot,                           false, 0},
    {NULL, NMA_Rebirth,             true,  METRIC_DATA_TYPE_BOOLEAN, &this->nodeRebirth,                          false, 0},
    {NULL, NMA_NextServer,          true,  METRIC_DATA_TYPE_BOOLEAN, &this->nodeNextServer,                       false, 0},
    {NULL, NMA_CommsVersion,        false, METRIC_DATA_TYPE_INT64,   &this->commsVersion,                         false, 0},
    {NULL, NMA_FirmwareVersion,     false, METRIC_DATA_TYPE_STRING,  &this->firmwareVersion,                      false, 0},
    {NULL, NMA_UniqueId,            false, METRIC_DATA_TYPE_UINT64,  &this->uniqueId,                             false, 0},
    {NULL, NMA_CommsChannel,        false, METRIC_DATA_TYPE_INT32,   &this->commsChannel,                         false, 0},
    {NULL, NMA_NTPSynchronized,     false, METRIC_DATA_TYPE_BOOLEAN, &this->ntpStatus.synchronized,               false, 0},
    {NULL, NMA_NTPLastUpdate,       false, METRIC_DATA_TYPE_UINT64,  &this->ntpStatus.last_update_time,           false, 0},
    {NULL, NMA_NTPServer,           false, METRIC_DATA_TYPE_STRING,  &this->ntpServerPtr,                         false, 0},
    {NULL, NMA_PanelStatus,         false, METRIC_DATA_TYPE_STRING,  &this->panelStatus,                          false, 0},
    {NULL, NMA_ActiveSchedule,      false, METRIC_DATA_TYPE_DATASET, &this->activeSchedule,                       false, 0},
    {NULL, NMA_ActiveScheduleStart, false, METRIC_DATA_TYPE_UINT64,  &this->dev.current_data.active_start_time,   false, 0},
    {NULL, NMA_CommandCounter,      false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.command_counter,     false, 0},
    {NULL, NMA_CommandIndex,        false, METRIC_DATA_TYPE_INT32,   &this->dev.current_data.command_index,       false, 0},
    {NULL, NMA_CommandSentNTP,      false, METRIC_DATA_TYPE_UINT64,  &this->dev.current_data.command_sent_ntp_ms, false, 0},
    {NULL, NMA_CommandSentMicros,   false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.command_sent_micros, false, 0},
    {NULL, NMA_HeaterOutput,        false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.heater_output,       false, 0},
    {NULL, NMA_VMon,                false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.v_mon,               false, 0},
    {NULL, NMA_ILMon,               false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.il_mon,              false, 0},
    {NULL, NMA_IRMon,               false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.ir_mon,              false, 0},
    {NULL, NMA_ErrorLamps,          false, METRIC_DATA_TYPE_UINT32,  &this->dev.error_lamps,                      false, 0},
    {NULL, NMA_CyclePeriod,         false, METRIC_DATA_TYPE_UINT32,  &this->dev.current_data.cycle_period,        false, 0},
    {NULL, NMA_CycleOffset,         false, METRIC_DATA_TYPE_INT32,   &this->dev.current_data.cycle_offset,        false, 0},
    {NULL, NMA_NextSchedule,        true,  METRIC_DATA_TYPE_DATASET, &this->nextSchedule,                         false, 0},
    {NULL, NMA_NextScheduleStart,   true,  METRIC_DATA_TYPE_UINT64,  &this->dev.current_data.next_start_time,     false, 0},
    {NULL, NMA_VDown,               true,  METRIC_DATA_TYPE_UINT32,  &this->dev.v_down,                           false, 0},
    {NULL, NMA_ILDown,              true,  METRIC_DATA_TYPE_UINT32,  &this->dev.il_down,                          false, 0},
    {NULL, NMA_IRDown,              true,  METRIC_DATA_TYPE_UINT32,  &this->dev.ir_down,                          false, 0},
#if defined(SIMULATED_POWER_PANEL)
    {NULL, NMA_Sim_NoReply,         true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.noReply,                          false, 0},
    {NULL, NMA_Sim_ParityErr,       true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.parityError,                      false, 0},
    {NULL, NMA_Sim_InvalidHex,      true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.invalidHex,                       false, 0},
    {NULL, NMA_Sim_ChecksumErr,     true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.checksumError,                    false, 0},
    {NULL, NMA_Sim_NoStrobe,        true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.noStrobe,                         false, 0},
    {NULL, NMA_Sim_ShortReply,      true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.shortReply,                       false, 0},
    {NULL, NMA_Sim_LongReply,       true,  METRIC_DATA_TYPE_BOOLEAN, &this->sim.longReply,                        false, 0},
    {NULL, NMA_Sim_VOn,             true,  METRIC_DATA_TYPE_UINT32,  &this->sim.v_on,                             false, 0},
    {NULL, NMA_Sim_VOff,            true,  METRIC_DATA_TYPE_UINT32,  &this->sim.v_off,                            false, 0},
    {NULL, NMA_Sim_ILOn,            true,  METRIC_DATA_TYPE_UINT32,  &this->sim.il_on,                            false, 0},
    {NULL, NMA_Sim_ILOff,           true,  METRIC_DATA_TYPE_UINT32,  &this->sim.il_off,                           false, 0},
    {NULL, NMA_Sim_IROn,            true,  METRIC_DATA_TYPE_UINT32,  &this->sim.ir_on,                            false, 0},
    {NULL, NMA_Sim_IROff,           true,  METRIC_DATA_TYPE_UINT32,  &this->sim.ir_off,                           false, 0},
    {NULL, NMA_Sim_ClockOffset,     true,  METRIC_DATA_TYPE_INT64,   &this->sim.clock_offset,                     false, 0},
#endif
  };

  // Fill in the metric name matching each alias
  for(unsigned int i = 0; i < NUM_NODE_METRICS; i++)
    s_NodeMetrics[i].name = MetricNames[s_NodeMetrics[i].alias];

  node_metrics.size = NUM_NODE_METRICS;
  node_metrics.metrics = s_NodeMetrics;

  // Check that the metric names are non-empty and the alias numbers in the
  // metrics are valid and unique
  int idx = 0;
  so_sp_err_t err = check_metrics(&node_metrics, EndNodeMetricAlias, &idx);
  if(err != SO_SP_ERR_NONE){
    DebugPrintNoEOL("Bad alias for metric #");
    DebugPrintNoEOL(idx);
    DebugPrintNoEOL(": ");
    DebugPrint(so_sp_get_err_txt(err));
    return false;
  }

  // Default settings
  IPAddress ip(PPI0_IP); // This device's IP address
  IPAddress dns(DNS); // DNS server
  IPAddress gateway(GATEWAY); // network gateway
  IPAddress subnet(SUBNET); // network subnet
  char dev_id = '0';

  // Set the MAC address
  byte mac[NUM_MAC_OCTETS];
  teensyMAC(mac);

  // Set the unique ID to the MAC address
  this->uniqueId = ((uint64_t) mac[0]) << 40 |
                   ((uint64_t) mac[1]) << 32 |
                   ((uint64_t) mac[2]) << 24 |
                   ((uint64_t) mac[3]) << 16 |
                   ((uint64_t) mac[4]) <<  8 |
                   ((uint64_t) mac[5]);

#if defined(DEBUG)
  {
    // Display the MAC address on the diagnostic port
    DebugPrintNoEOL("MAC: ");
    char hex_string[3 * NUM_MAC_OCTETS + 1] = { '\0' };
    char *next = &hex_string[0];
    for(unsigned octet = 0; octet < NUM_MAC_OCTETS; octet++){
      uint8_t nibble = ( mac[octet] >> 4 ) & 0x0F;
      *next = nibble < 10 ? nibble + '0' : nibble - 10 + 'A';
      next++;
      nibble = ( mac[octet] ) & 0x0F;
      *next = nibble < 10 ? nibble + '0' : nibble - 10 + 'A';
      next++;
      *next = ':';
      next++;
    }
    next--;
    *next = '\0';
    DebugPrint(hex_string);
  }
#endif

  // Read the hardware ID from the jumpers
  int jumpers = (int)PPI_JP1;
  jumpers += ((int)PPI_JP2) << 1;
  jumpers += ((int)PPI_JP3) << 2;
  jumpers += ((int)PPI_JP4) << 3;

  DebugPrintNoEOL("Hardware ID: ");
  DebugPrint(jumpers);

  if(jumpers < 0 || jumpers >= NUM_PANELS){
    // Jumpers are not set for a valid power panel - abort
    DebugPrintNoEOL("Invalid Hardware ID, must be >= 0 and <= ");
    DebugPrint(NUM_PANELS-1);
    return false;
  }

  int ppi_id = jumpers;

  // Adjust the network settings to the PPI ID
  dev_id += ppi_id;

  // Adjust the last octet of the IP for PPI
  ip[3] += ppi_id;

  DebugPrintNoEOL("My IP address: ");
  DebugPrint(ip);

  // Set up the Ethernet
  this->network_lock = net_lock;
  lock_network();
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  this->enet.setConnectionTimeout(BROKER_CONNECT_TIMEOUT_MS);
  this->broker.setClient(this->enet);
  this->broker.setCallback(callback);
  this->broker.setBufferSize(sizeof(buffer));
  // Limit the transfer size so big MQTT messages don't exceed socket buffer size
  this->broker.setMaxTransferSize(80);
  unlock_network();

  generateNames(dev_id);

  // Force an update of all the published metrics
  update_status(true);

  // Success
  return true;
}

// Lock the network to prevent other threads from accessing it.
void PowerPanelInterface::lock_network(void){
  if(this->network_lock != NULL)
    this->network_lock->lock();
}

// Unlock the network to allow other threads to access it.
void PowerPanelInterface::unlock_network(void){
  if(this->network_lock != NULL)
    this->network_lock->unlock();
}

// Set up a schedule dataset to be able to hold a full set of power commands.
bool PowerPanelInterface::set_up_schedule_dataset(dataset_t *schedule, unsigned int max_commands){
  if(schedule == NULL){
    DebugPrint("Null pointer");
    return false;
  }

  schedule->has_num_of_columns = true;
  schedule->has_num_of_columns = true;
  schedule->num_of_columns = 1;
  schedule->columns_count = 1;
  schedule->columns = (char **) calloc(schedule->columns_count,
                                       sizeof(*schedule->columns));
  if(schedule->columns == NULL){
    DebugPrintNoEOL("Memory alloc failed for columns: ");
    DebugPrint(schedule->columns_count * sizeof(*schedule->columns));
    return false;
  }
  schedule->columns[0] = (char *) "Outputs";
  schedule->types_count = 1;
  schedule->types = (uint32_t *) calloc(schedule->types_count,
                                        sizeof(*schedule->types));
  if(schedule->types == NULL){
    DebugPrintNoEOL("Memory alloc failed for types: ");
    DebugPrint(schedule->types_count * sizeof(*schedule->types));
    return false;
  }
  schedule->types[0] = DATA_SET_DATA_TYPE_UINT32;
  schedule->rows_count = 0;  // Currently contains no rows
  schedule->rows = (struct _org_eclipse_tahu_protobuf_Payload_DataSet_Row *)
                        calloc(max_commands, sizeof(*schedule->rows));
  if(schedule->rows == NULL){
    DebugPrintNoEOL("Memory alloc failed for rows: ");
    DebugPrint(max_commands * sizeof(*schedule->rows));
    return false;
  }
  if(max_commands > 0){
    // Allocate all element memory in a single block
    dataset_value_t *elements = (dataset_value_t *) calloc(max_commands,
                                                           sizeof(*elements));
    if(elements == NULL){
      DebugPrintNoEOL("Memory alloc failed for ");
      DebugPrintNoEOL(max_commands);
      DebugPrintNoEOL("elements: ");
      DebugPrint(max_commands * sizeof(*elements));
      return false;
    }
    for(unsigned int i = 0; i < max_commands; i++){
      schedule->rows[i].elements_count = 1;
      schedule->rows[i].elements = &elements[i];
      schedule->rows[i].elements[0].which_value = org_eclipse_tahu_protobuf_Payload_DataSet_DataSetValue_int_value_tag;
      schedule->rows[i].elements[0].value.int_value = 0;
      schedule->rows[i].extensions = NULL;
    }

  }
  schedule->extensions = NULL;

  // Success
  return true;
}

// Copy the scheduled heater output list to the schedule metric variable.
// Return true if the metric variable has been changed.
bool PowerPanelInterface::copy_schedule(uint32_t outs[], uint32_t rows_count, dataset_t *schedule){
  // Check input parameters are valid
  if(schedule == NULL){
    DebugPrint("Null schedule pointer");
    return false;
  }
  if(outs == NULL && rows_count != 0){
    DebugPrint("Null outs pointer");
    return false;
  }

  // Copy the outputs to the dataset array
  bool different = false;
  if(schedule->rows_count != rows_count){
    schedule->rows_count = rows_count;
    different = true;
  }
  for(unsigned int i = 0; i < rows_count; i++)
    if(schedule->rows[i].elements[0].value.int_value != outs[i]){
      schedule->rows[i].elements[0].value.int_value = outs[i];
      different = true;
    }

  // Return true if anything changed
  return different;
}

// Release memory used by a schedule dataset.
void PowerPanelInterface::free_schedule(dataset_t *schedule){
  if(schedule == NULL)
    // Invalid pointer - nothing to do
    return;

  if(schedule->columns != NULL){
    free(schedule->columns);
    schedule->columns = NULL;
  }
  if(schedule->types != NULL){
    free(schedule->types);
    schedule->types = NULL;
  }
  if(schedule->rows != NULL){
    // All element memory is in a single block pointed to by the first element
    if(schedule->rows[0].elements != NULL){
      free(schedule->rows[0].elements);
      schedule->rows[0].elements = NULL;
    }
    free(schedule->rows);
    schedule->rows = NULL;
  }
}

// Sets the name of topics and the node ID based on the module ID.
void PowerPanelInterface::generateNames(char dev_id){
  // Sparkplug node and topic names
  this->node_id = NODE_ID_TEMPLATE;
  this->nodeBirthTopic = NODE_TOPIC(NBIRTH_MESSAGE_TYPE, NODE_ID_TEMPLATE);
  this->nodeDeathTopic = NODE_TOPIC(NDEATH_MESSAGE_TYPE, NODE_ID_TEMPLATE);
  this->nodeDataTopic  = NODE_TOPIC(NDATA_MESSAGE_TYPE,  NODE_ID_TEMPLATE);
  this->nodeCmdTopic   = NODE_TOPIC(NCMD_MESSAGE_TYPE,   NODE_ID_TEMPLATE);

  this->node_id.replace(NODE_ID_TOKEN, dev_id);
  this->nodeBirthTopic.replace(NODE_ID_TOKEN, dev_id);
  this->nodeDeathTopic.replace(NODE_ID_TOKEN, dev_id);
  this->nodeDataTopic.replace(NODE_ID_TOKEN, dev_id);
  this->nodeCmdTopic.replace(NODE_ID_TOKEN, dev_id);
}

// Connect to a broker and send out initial messages.
bool PowerPanelInterface::connect_to_broker(){
  // Increment the birth/death sequence number before creating the NDEATH
  // message
  this->bdSeq++;

  // Create the NDEATH message with its metrics
  set_up_ndeath_payload(&sp, bdSeq);
  if(sp_encode_obj(&sp)){
    DebugPrint(so_sp_get_err_txt(SO_SP_ERR_ENCODEFAILED));
    DebugPrint("Failed to encode NDEATH");
    this->bdSeq--;
    return false;
  }

  lock_network();
  if(broker_index==0)
    broker.setServer(IPAddress(MQTT_BROKER_1), MQTT_BROKER_1_PORT);
  else
    broker.setServer(IPAddress(MQTT_BROKER_2), MQTT_BROKER_2_PORT);

  // Connect to the broker, with the NDEATH message as our "will"
  bool connected = broker.connect(node_id.c_str(), nodeDeathTopic.c_str(), 0,
                                  false, sp.buffer, sp.encoded_len);
  unlock_network();

  if(!connected){
    DebugPrintNoEOL("Failed to connect to broker");
    DebugPrint(broker_index+1);
    this->bdSeq--;
    return false;
  }

  // Subscribe to the topics we're interested in
  if(!subscribeTopics()){
    DebugPrint("Unable to subscribe to topics on broker");
    // Disconnect gracefully from the broker
    disconnect_from_broker();
    return false;
  }

  // Success
  return true;
}

// Subscribe to the required topics on the given broker.
bool PowerPanelInterface::subscribeTopics(){
  bool success = true;
  lock_network();
  if(!broker.subscribe(FCC_HOST_STATE_TOPIC))
    success = false;
  if(!broker.subscribe(this->nodeCmdTopic.c_str()))
    success = false;
  unlock_network();
  return success;
}

/**
@brief Graceful disconnect from the current broker
This function sends the NDEATH message then gracefully disconnects from the
currently connected broker.
*/
void PowerPanelInterface::disconnect_from_broker(){
  set_up_ndeath_payload(&sp, bdSeq);
  if(sp_encode_obj(&sp)){
    DebugPrint("Failed to encode NDEATH for disconnect");
  }
  lock_network();
  broker.publish(nodeDeathTopic.c_str(), sp.buffer, sp.encoded_len, false);
  broker.disconnect();
  unlock_network();

  // Primary Host is offline until we get a new host state message
  primary_host_online = false;
}

// Check a broker is connected, and if not then attempt to connect to one.
// Keep the connection to the broker open, process incoming MQTT messages, and
// publish birth and data messages as necessary.  Returns true if a broker is
// connected; otherwise false.  This function should be called periodically.
bool PowerPanelInterface::check_broker(void){
  lock_network();
  bool is_connected = this->broker.connected();
  unlock_network();
  if(!is_connected){
    // Primary Host is offline until we get a new host state message
    primary_host_online = false;

    // Discard any messages that were waiting to be published
    publish_requested = false;

    // If we didn't just lose our connection and we've tried to connect to this
    // broker recently, wait before attempting to reconnect.  Note that this
    // calculation handles wraparound of broker_retry_timeout_start or millis().
    if(!was_connected && broker_index == last_broker_index &&
       (uint32_t) (millis() - broker_retry_timeout_start) <= BROKER_RECONNECT_TIMEOUT_MS)
      return false;

    was_connected = false;

    // Try to connect to the broker
    if(!connect_to_broker()){
      // Can't connect - move to the next broker
      broker_index = broker_index ? 0 : 1;

      // Start the timeout for retrying the broker connections
      broker_retry_timeout_start = (uint32_t) millis();
      return false;
    }

    was_connected = true;
    last_broker_index = broker_index;

    DebugPrintNoEOL("Connected to broker");
    DebugPrint(broker_index+1);

    // Wait for a while before looking for the Primary Host on the next broker
    primary_host_timeout_start = (uint32_t) millis();
    if(first_broker_connection){
      // It may take a second or two to receive a subscribed Host STATE message
      first_broker_connection = false;
      primary_host_timeout = FIRST_SWITCH_TIMEOUT_MS;
    }
    else
      primary_host_timeout = PRIMARY_HOST_TIMEOUT_MS;
  }

  // Handle any incoming messages, as well as maintaining our connection to
  // the broker
  lock_network();
  this->broker.loop();
  unlock_network();

  // Move to the next broker if the Primary Host has been offline for too long.
  // Note that this calculation handles wraparound of primary_host_timeout_start
  // or millis().
  if(!primary_host_online &&
     (uint32_t) (millis() - primary_host_timeout_start) > primary_host_timeout){
    this->nodeNextServer = true;
    DebugPrint("Timed out waiting for primary host, trying next server");
  }

  // Have we been asked to move to the next broker?
  if(this->nodeNextServer){
    this->nodeNextServer = false;
    disconnect_from_broker();
    broker_index = broker_index ? 0 : 1;
  }
  // Have we been asked to re-publish our birth messages?
  else if(this->nodeRebirth){
    this->nodeRebirth = false;
    publish_births();
  }
  else{
    // Publish any Node data that has changed IF we didn't just send births
    publish_node_data();
  }

  // Return true if a broker is connected
  lock_network();
  is_connected = this->broker.connected();
  unlock_network();
  return is_connected;
}

// Publish the NBIRTH message with all metrics specified.
void PowerPanelInterface::publish_births(void){
  // Create and publish the NBIRTH message containing the bdSeq metric
  // together with all the node metrics
  set_up_nbirth_payload(&sp, bdSeq);
  if(add_metrics(&sp, true, &node_metrics)){
    DebugPrint("Failed to add metrics to NBIRTH");
  }
  else if(sp_encode_obj(&sp)){
    DebugPrint("Failed to encode NBIRTH");
  }
  else{
    lock_network();
    if(!broker.publish(nodeBirthTopic.c_str(), sp.buffer, sp.encoded_len, false))
      DebugPrint("Failed to publish NBIRTH");
    unlock_network();
  }

  // The NBIRTH message will have included any pending data
  publish_requested = false;
}

// Publish the NDATA message with any node metrics that have been updated.
void PowerPanelInterface::publish_node_data(void){
  // Publish any updated metrics in the NDATA message
  lock_network();
  bool is_connected = this->broker.connected();
  unlock_network();
  if(!is_connected || !publish_requested || !primary_host_online)
    return;

  set_up_next_payload(&sp);
  if(add_metrics(&sp, false, &node_metrics)){
    DebugPrint("Failed to add metrics to NDATA");
  }
  else if(sp_encode_obj(&sp)){
    DebugPrint("Failed to encode NDATA");
  }
  else{
    lock_network();
    if(!broker.publish(this->nodeDataTopic.c_str(), sp.buffer, sp.encoded_len))
      if(broker.connected())
        DebugPrint("Failed to publish NDATA");
    unlock_network();
  }

  this->publish_requested = false;
}

// Callback registered with broker to handle incoming MQTT messages.
//   topic - the subscribed topic that we're getting data from.
//   payload - the incoming payload we're receiving on that topic.
//   len - the length, in bytes, of the payload.
void PowerPanelInterface::callback_worker(char* topic, byte* payload, unsigned int len){
  // Check parameters are valid
  if(topic == NULL || strcmp(topic, "") == 0){
    // Topic was not specified - don't do anything
    DebugPrint("No topic specified");
    return;
  }
  if(payload == NULL){
    // Payload was not specified - don't do anything
    DebugPrint("No payload specified");
    return;
  }
  if(len == 0){
    // Payload is empty - don't do anything
    DebugPrint("Payload length is zero");
    return;
  }

  // Process the message according to its type
  if(!process_host_state_message(topic, payload, len) &&
     !process_node_cmd_message(topic, payload, len)){
    // Unrecognized message
    char topic_short[40];
    snprintf(topic_short, sizeof(topic_short), "%s", topic);
    DebugPrintNoEOL("Unrecognized message topic: \"");
    DebugPrintNoEOL(topic_short);
    DebugPrint("\"");
  }
}

// Check to see if a received message is a Host STATE message.  If it is,
// handle it and return true, even if it's invalid; otherwise return false.
bool PowerPanelInterface::process_host_state_message(char* topic, byte* payload, unsigned int len){
  if(strcmp(topic, FCC_HOST_STATE_TOPIC) != 0)
    // This is not a Host STATE message
    return false;

  if(len >= MAX_HOST_STATE_MSG_LEN){
    DebugPrint("Ignored a host state message - too long");
    return true;  // This was a Host STATE message
  }

  payload[len]=0; // Add string termination character to the end

  // These messages are JSON encoded, so deserialize
  DeserializationError err = deserializeJson(json, payload);
  if(err){
    DebugPrintNoEOL("Host state message parsing failed: ");
    DebugPrint(err.c_str());
    return true;  // This was a Host STATE message
  }

  JsonVariant online = json[STATE_ONLINE_KEY];
  JsonVariant statetime = json[STATE_TIMESTAMP_KEY];

  // Ignore messages with a missing or invalid timestamp
  if(!statetime.is<uint64_t>()){
    DebugPrint("Ignored a host state message - missing/invalid timestamp");
    return true;  // This was a Host STATE message
  }

  // Ignore old state messages
  uint64_t msg_statetime = statetime;
  if(msg_statetime < primary_host_statetime){
    DebugPrint("Ignored a host state message - too old");
    return true;  // This was a Host STATE message
  }

  bool primary_host_was_online = primary_host_online;
  primary_host_statetime = msg_statetime;
  if(online.is<bool>())
    primary_host_online = online;
  else{
    // If online is missing or invalid assume host is offline
    primary_host_online = false;
    DebugPrint("Invalid host state message - missing/invalid online");
  }

  // We send birth certificates if we see a primary host come online
  nodeRebirth |= primary_host_online;

  if(!primary_host_online){
    if(primary_host_was_online){
      // Only update the timeout if the primary host had been online.  This
      // prevents repeated STATE offline messages from updating the timeout.
      primary_host_timeout_start = (uint32_t) millis();
      primary_host_timeout = PRIMARY_HOST_TIMEOUT_MS;
    }

    // Primary Host is not connected to this broker
    DebugPrint("Primary Host " FCC_HOST " is OFFLINE");
  }
  else
    // Primary Host is connected to this broker
    DebugPrint("Primary Host " FCC_HOST " is ONLINE");

  return true;  // This was a Host STATE message
}

// Check to see if a received message is a Node command (NCMD) message.  If it
// is, handle it and return true, even if it's invalid; otherwise return false.
bool PowerPanelInterface::process_node_cmd_message(char* topic, byte* payload, unsigned int len){
  if(strcmp(topic, this->nodeCmdTopic.c_str()) != 0)
    // This is not a Node command message
    return false;

  // Decode the Sparkplug payload
  if(!sp_decode(&rx_payload, payload, len)){
    // Invalid payload - don't do anything
    sp_free_payload(&rx_payload);
    DebugPrint("Unable to decode Node command payload");
    // This was a Node command message
    return true;
  }

  // Track whether we received any schedule metrics in this message
  bool scheduleSet = false;
  uint32_t num_elements = 0;
  uint32_t elements[PPLIST_MAX_ELEMENTS] = { 0 };
  bool scheduleStartSet = false;
  uint64_t scheduleStart = 0;

  // Process the metrics
  for(unsigned int idx = 0; idx < rx_payload.metrics_count; idx++){
    metric_t *metric = &rx_payload.metrics[idx];
    if(!metric->has_alias){
      // Missing alias - skip this metric
      DebugPrint("Metric does not have an alias");
      continue;
    }

    metric_spec_t *spec = NULL;
    so_sp_err_t err = find_metric_by_alias(&spec, &node_metrics, metric->alias);
    if(err != SO_SP_ERR_NONE || spec == NULL){
      // Invalid metric - skip it
      DebugPrintNoEOL("Unrecognized Node metric alias: ");
      DebugPrint(metric->alias);
      continue;
    }

    // Macro to update the metric for a data field if it changed
    #define UPDATE_METRIC(field, new_value)  \
      do { \
        if(this->field != new_value){ \
          this->field = new_value; \
          if(update_metric(&node_metrics, &this->field, ntp_get_current_time_millis()) == SO_SP_ERR_NONE) \
            publish_requested = true; \
          else \
            DebugPrint("Failed to update " #field " metric"); \
        } \
      } while(0)

    // Now handle the metric
    int64_t alias = spec->alias;
    switch(alias){
    case NMA_Reboot:
      if(metric->value.boolean_value){
        DebugPrint("Reboot command received");
        // Reboot immediately - don't attempt to process the rest of
        // the message, publish data, send death certificate,
        // disconnect from broker, or close the network
        reset_teensy();
      }
      break;

    case NMA_Rebirth:
      if(metric->value.boolean_value){
        // Publish birth messages again
        this->nodeRebirth = true;
        DebugPrint("Node Rebirth command received");
      }
      break;

    case NMA_NextServer:
      if(metric->value.boolean_value){
        // Move to the next broker
        this->nodeNextServer = true;
        DebugPrint("NextServer command received");
      }
      break;

    case NMA_NextSchedule:
    {
      // Set the power panel's next heater output list
      dataset_t *d = &metric->value.dataset_value;
      if(d->has_num_of_columns &&
         d->num_of_columns == 1 &&
         d->rows_count >= 0 &&
         d->rows_count <= PPLIST_MAX_ELEMENTS &&
         (d->types[0] == DATA_SET_DATA_TYPE_INT32 ||
          d->types[0] == DATA_SET_DATA_TYPE_UINT32)){
        num_elements = (uint32_t) d->rows_count;
        for(unsigned int i = 0; i < num_elements; i++)
          elements[i] = heaters_only(d->rows[i].elements[0].value.int_value);
        scheduleSet = true;
      }
      break;
    }

    case NMA_NextScheduleStart:
      // Set the starting time of the next heater output list (but only if the
      // start time is provided in the same message as a new schedule)
      scheduleStart = metric->value.long_value;
      scheduleStartSet = true;
      break;

    case NMA_VDown:
      // Set which voltage sensors are down to block known bad sensors from
      // generating errors
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(dev.v_down, heaters_only(metric->value.int_value));
      break;

    case NMA_ILDown:
      // Set which local current sensors are down to block known bad sensors
      // from generating errors
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(dev.il_down, heaters_only(metric->value.int_value));
      break;

    case NMA_IRDown:
      // Set which remote current sensors are down to block known bad sensors
      // from generating errors
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(dev.ir_down, heaters_only(metric->value.int_value));
      break;

#if defined(SIMULATED_POWER_PANEL)
    case NMA_Sim_NoReply:
      UPDATE_METRIC(sim.noReply, metric->value.boolean_value);
      this->ppsim.noReply(this->sim.noReply);
      break;

    case NMA_Sim_ParityErr:
      UPDATE_METRIC(sim.parityError, metric->value.boolean_value);
      this->ppsim.parityError(this->sim.parityError);
      break;

    case NMA_Sim_InvalidHex:
      UPDATE_METRIC(sim.invalidHex, metric->value.boolean_value);
      this->ppsim.invalidHex(this->sim.invalidHex);
      break;

    case NMA_Sim_ChecksumErr:
      UPDATE_METRIC(sim.checksumError, metric->value.boolean_value);
      this->ppsim.checksumError(this->sim.checksumError);
      break;

    case NMA_Sim_NoStrobe:
      UPDATE_METRIC(sim.noStrobe, metric->value.boolean_value);
      this->ppsim.noStrobe(this->sim.noStrobe);
      break;

    case NMA_Sim_ShortReply:
      UPDATE_METRIC(sim.shortReply, metric->value.boolean_value);
      this->ppsim.shortReply(this->sim.shortReply);
      break;

    case NMA_Sim_LongReply:
      UPDATE_METRIC(sim.longReply, metric->value.boolean_value);
      this->ppsim.longReply(this->sim.longReply);
      break;

    case NMA_Sim_VOn:
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(sim.v_on, heaters_only(metric->value.int_value));
      this->ppsim.v_on(this->sim.v_on);
      break;

    case NMA_Sim_VOff:
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(sim.v_off, heaters_only(metric->value.int_value));
      this->ppsim.v_off(this->sim.v_off);
      break;

    case NMA_Sim_ILOn:
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(sim.il_on, heaters_only(metric->value.int_value));
      this->ppsim.il_on(this->sim.il_on);
      break;

    case NMA_Sim_ILOff:
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(sim.il_off, heaters_only(metric->value.int_value));
      this->ppsim.il_off(this->sim.il_off);
      break;

    case NMA_Sim_IROn:
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(sim.ir_on, heaters_only(metric->value.int_value));
      this->ppsim.ir_on(this->sim.ir_on);
      break;

    case NMA_Sim_IROff:
      // Mask out any bits not associated with heaters
      UPDATE_METRIC(sim.ir_off, heaters_only(metric->value.int_value));
      this->ppsim.ir_off(this->sim.ir_off);
      break;

    case NMA_Sim_ClockOffset:
      UPDATE_METRIC(sim.clock_offset, (int64_t) metric->value.long_value);
      ntp_set_offset(this->sim.clock_offset);
      break;
#endif

    default:
      DebugPrintNoEOL("Unhandled Node metric alias: ");
      DebugPrint(alias);
      break;
    }
  }

  if(scheduleSet)
    // Register the new schedule
    // Note that a schedule start time is only applied if it was provided in
    // the same message as a new schedule.
    // Note that we don't publish the new metrics until they've been accepted
    // by the panel.
    (void)this->apply_next_schedule_(elements, num_elements, scheduleStartSet,
                                     scheduleStart);

  // Free the decoder memory
  sp_free_payload(&rx_payload);

  // This was a Node command message
  return true;
}

// Mask out any bits that are not associated with heaters.
uint32_t PowerPanelInterface::heaters_only(uint32_t heater_word){
  return heater_word & 0x3FFFFFFF;
}

// Definitions for resetting Teensy
#if defined(LINUX_SIMULATOR)
  #define WRITE_RESTART(val) exit(0)
#else
  #ifndef RESTART_ADDR
  #define RESTART_ADDR 0xE000ED0C
  #endif
  #define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
  #define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#endif

// Software reset the Teensy.
void PowerPanelInterface::reset_teensy(void){
  WRITE_RESTART(0x5FA0004);
}

// Retrieve the current status from the power panel and NTP and publish any
// metrics that have changed.  Returns true if any of the data has changed;
// otherwise returns false.
bool PowerPanelInterface::update_status(bool force){
  bool changed = false;

  // Set the update time for any metrics that changed to the current time
  uint64_t update_time = ntp_get_current_time_millis();

  // Get current NTP status
  ntp_status_t newNtpStatus;
  ntp_get_status(&newNtpStatus);

  // Update the NTP Synchronized metric if it changed
  if(force || newNtpStatus.synchronized != this->ntpStatus.synchronized){
    this->ntpStatus.synchronized = newNtpStatus.synchronized;
    if(update_metric(&node_metrics, &this->ntpStatus.synchronized, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update NTP Synchronized metric");
    changed = true;
  }

  // Update the NTP Last Update Time metric if it changed
  if(force || newNtpStatus.last_update_time != this->ntpStatus.last_update_time){
    this->ntpStatus.last_update_time = newNtpStatus.last_update_time;
    if(update_metric(&node_metrics, &this->ntpStatus.last_update_time, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update NTP Last Update Time metric");
    changed = true;
  }

  // Update the NTP Server metric if it changed
  if(force || newNtpStatus.server_ip != this->ntpStatus.server_ip){
    if(newNtpStatus.server_ip != NULL)
      snprintf(this->ntpServer, sizeof(this->ntpServer), "%u.%u.%u.%u",
               (unsigned) (*newNtpStatus.server_ip)[0],
               (unsigned) (*newNtpStatus.server_ip)[1],
               (unsigned) (*newNtpStatus.server_ip)[2],
               (unsigned) (*newNtpStatus.server_ip)[3]);
    else
      snprintf(this->ntpServer, sizeof(this->ntpServer), "NULL");
    if(update_metric(&node_metrics, &this->ntpServerPtr, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update NTP Server metric");
    changed = true;
  }

  // Update our copy of the NTP data
  memcpy(&this->ntpStatus, &newNtpStatus, sizeof(this->ntpStatus));

  // Snapshot current panel state from the direct I/O backend.
  snapshot_panel_state_();

  // Compute the error indicator lamp settings
  uint32_t error_lamps = 0;
  Serial.println("Panel status: " + String(this->dev.current_data.panel_status));
  if(this->dev.current_data.panel_status == PPI_PANEL_STATUS_IDLE ||
     this->dev.current_data.panel_status == PPI_PANEL_STATUS_ACTIVE)
  { // no error
    uint32_t ho = this->dev.current_data.heater_output & HEATER_MASK_30;
    // Serial.println("ho: " + String(ho, BIN));
    uint32_t v  = this->dev.current_data.v_mon & HEATER_MASK_30;
    // Serial.println("v: " + String(v, BIN));
    uint32_t il = this->dev.current_data.il_mon & HEATER_MASK_30;
    // Serial.println("il: " + String(il, BIN));
    uint32_t ir = this->dev.current_data.ir_mon & HEATER_MASK_30;
    // Serial.println("ir: " + String(ir, BIN));

    // Healthy sensors are those not flagged as known-down by host overrides.
    const uint32_t v_good  = (~this->dev.v_down)  & HEATER_MASK_30;
    const uint32_t il_good = (~this->dev.il_down) & HEATER_MASK_30;
    const uint32_t ir_good = (~this->dev.ir_down) & HEATER_MASK_30;

    // For each heater bit:
    // - any_sensor_on  catches HO=0 mismatches (failed-on indication)
    // - any_sensor_off catches HO=1 mismatches (failed-off indication)
    const uint32_t any_sensor_on = (v & v_good) | (il & il_good) | (ir & ir_good);
    const uint32_t any_sensor_off = ((~v) & v_good) | ((~il) & il_good) | ((~ir) & ir_good);

    const uint32_t ho_off_mismatch = ((~ho) & HEATER_MASK_30) & any_sensor_on;
    const uint32_t ho_on_mismatch = ho & any_sensor_off;

    // Serial.println("any_sensor_on: " + String(any_sensor_on, BIN));
    // Serial.println("any_sensor_off: " + String(any_sensor_off, BIN));
    // Serial.println("ho_off_mismatch: " + String(ho_off_mismatch, BIN));
    // Serial.println("ho_on_mismatch: " + String(ho_on_mismatch, BIN));

    error_lamps = (ho_off_mismatch | ho_on_mismatch) & HEATER_MASK_30;
  }
  else
  {
    // Communications error - turn all error lamps on
    error_lamps = 0xFFFFFFFF;
  }
  Serial.println("Error lamps: " + String(error_lamps, BIN));

  // Update the error lamps metric if it changed
  if(force || this->dev.error_lamps != error_lamps){
    this->dev.error_lamps = error_lamps;
    if(update_metric(&node_metrics, &this->dev.error_lamps, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update Error Lamps metric");
  }

  // Update the power panel status metric if it changed
  if(force || this->dev.current_data.panel_status != this->dev.last_data.panel_status){
    // Convert power panel status to a string
    this->panelStatus = panelStatusToString_(this->dev.current_data.panel_status);
    if(update_metric(&node_metrics, &this->panelStatus, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update Panel Status metric");
  }

  // Macro to update the metric for a data field if it has changed
  #define UPDATE_DEV_METRIC(field)  \
    do { \
      if(force || this->dev.current_data.field != this->dev.last_data.field){ \
        if(update_metric(&node_metrics, &this->dev.current_data.field, update_time) == SO_SP_ERR_NONE) \
          publish_requested = true; \
        else \
          DebugPrint("Failed to update " #field " metric"); \
      } \
    } while(0)

  // Update any changed metrics
  UPDATE_DEV_METRIC(command_counter);
  UPDATE_DEV_METRIC(active_start_time);
  UPDATE_DEV_METRIC(next_start_time);
  UPDATE_DEV_METRIC(command_index);
  UPDATE_DEV_METRIC(command_sent_ntp_ms);
  UPDATE_DEV_METRIC(command_sent_micros);
  UPDATE_DEV_METRIC(heater_output);
  UPDATE_DEV_METRIC(v_mon);
  UPDATE_DEV_METRIC(il_mon);
  UPDATE_DEV_METRIC(ir_mon);
  UPDATE_DEV_METRIC(cycle_period);
  UPDATE_DEV_METRIC(cycle_offset);

  if(copy_schedule(this->dev.current_data.active_schedule,
                   (pb_size_t) this->dev.current_data.active_schedule_size,
                   &this->activeSchedule) || force){
    if(update_metric(&node_metrics, &this->activeSchedule, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update Active Schedule metric");
  }

  if(copy_schedule(this->dev.current_data.next_schedule,
                   (pb_size_t) this->dev.current_data.next_schedule_size,
                   &this->nextSchedule) || force){
    if(update_metric(&node_metrics, &this->nextSchedule, update_time) == SO_SP_ERR_NONE)
      publish_requested = true;
    else
      DebugPrint("Failed to update Next Schedule metric");
  }

  const bool dataChanged = changed ||
                           (memcmp(&this->dev.current_data,
                                   &this->dev.last_data,
                                   sizeof(this->dev.current_data)) != 0);

  // Update last data
  memcpy(&this->dev.last_data, &this->dev.current_data, sizeof(this->dev.current_data));

  // Don't publish data if this was a forced update
  if(force)
    publish_requested = false;

  // Data has changed
  return dataChanged;
}

// Return true if the Primary Host is online on the broker; return false if the
// Primary Host is not online or we're not connected to a broker.
bool PowerPanelInterface::is_primary_host_online(void){
  if(!primary_host_online)
    return false;
  lock_network();
  bool is_connected = this->broker.connected();
  unlock_network();
  return is_connected;
}
