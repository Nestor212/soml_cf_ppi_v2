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
@brief SparkplugB, MQTT, and Network for the power panel interfaces
@author Michael Sibayan
@date January 27, 2021
@file ppi.hpp

This file contains all of the possible IP addresses used by power panel
interfaces.
*/

#ifndef __PPI_H__
#define __PPI_H__


#include <Arduino.h>
#include <NativeEthernet.h> // Teensy requires NativeEthernet to use onboard NIC
#include <PubSubClient.h> // pub&sub MQTT messages
#include "so_sparkplugb.h"
#include "ppi_global.hpp"
#include "ppi_metrics.hpp"
#include "ntp_thread.hpp"
#include "pp.hpp"

#if defined(SIMULATED_POWER_PANEL)
#include "pp_sim.hpp"
#endif


using namespace PPI;


// Common network configuration values
#define NUM_PANELS   9
#define NUM_BROKERS  2

#define NUM_ELEM(array)   (sizeof(array) / sizeof(*array))

#if defined(DEVELOPMENT_SYSTEM)
  // IP addresses for the Development system
  #define GATEWAY             192, 168, 1, 1
  #define SUBNET              255, 255, 255, 0
  #define DNS                 192, 168, 1, 1
  #define MQTT_BROKER_1       192, 168, 1, 91
  #define MQTT_BROKER_2       192, 168, 1, 92
  #define MQTT_BROKER_1_PORT  1883
  #define MQTT_BROKER_2_PORT  1883
  // Base IP address (final octet is incremented by power panel number).
  // This module has been allotted the range 192.168.1.150-158.
  #define PPI0_IP             192, 168, 1, 150
#else
  // IP addresses for the Production system
  #define GATEWAY             192, 168, 1, 1
  #define SUBNET              255, 255, 255, 0
  #define DNS                 192, 168, 1, 1
  #define MQTT_BROKER_1       192, 168, 1, 51
  #define MQTT_BROKER_2       192, 168, 1, 52
  #define MQTT_BROKER_1_PORT  1883
  #define MQTT_BROKER_2_PORT  1883
  // Base IP address (final octet is incremented by power panel number).
  // This module has been allotted the range 192.168.1.150-158.
  #define PPI0_IP             192, 168, 1, 150
#endif

// Pin assignments
#define JUMPER_PIN_1 28
#define JUMPER_PIN_2 29
#define JUMPER_PIN_3 30
#define JUMPER_PIN_4 31
#define JUMPER_PIN_5 32

// Jumper pins have inverted sense, so reverse the value read
#define PPI_JP1 (!digitalRead(JUMPER_PIN_1))
#define PPI_JP2 (!digitalRead(JUMPER_PIN_2))
#define PPI_JP3 (!digitalRead(JUMPER_PIN_3))
#define PPI_JP4 (!digitalRead(JUMPER_PIN_4))
#define PPI_JP5 (!digitalRead(JUMPER_PIN_5))

#define FCC_HOST  "FCC" // Furnace control host
#define FCC_HOST_STATE_TOPIC  STATE_TOPIC "/" FCC_HOST // Furnace control host state
#define MAX_HOST_STATE_MSG_LEN 256 // ignore state data longer than this
#define BROKER_CONNECT_TIMEOUT_MS 1000 // wait this long when trying to connect to a broker (msec)
#define BROKER_RECONNECT_TIMEOUT_MS 2000 // wait this long before retrying a broker connection (msec)
#define FIRST_SWITCH_TIMEOUT_MS  2000 // wait this long before switching brokers the first time (msec)
#define PRIMARY_HOST_TIMEOUT_MS 30000 // wait this long before switching brokers (msec)
#define MAX_SPARKPLUG_METRICS  EndNodeMetricAlias
#define NUM_NODE_METRICS  (EndNodeMetricAlias-1) // number of Node metrics, not including bdseq
#define BIN_BUF_SIZE 10000 // size of buffer used to construct outgoing messages (bytes)


/**
@brief helper struct for all power panel related data
*/
typedef struct power_panel_device_data{
  power_panel_data_t current_data;
  power_panel_data_t last_data;
  uint32_t error_lamps;
  uint32_t v_down;
  uint32_t il_down;
  uint32_t ir_down;
} power_panel_device_data_t;

/** @brief MQTT & Sparkplug B networking interface for a power panel */
class PowerPanelInterface{
public:
  PowerPanelInterface(); //!< @brief constructor
  ~PowerPanelInterface(); //!< @brief destructor

  bool start_panel(int main_port); //!< @brief Start up the panel thread/s
  bool network_init(Threads::Mutex *net_lock); //!< @brief Initialize the network
  bool check_broker(void); //!< @brief Check connection to broker
  /**
  @brief Handle an incoming MQTT message.
  @param topic the message topic
  @param payload data in the message payload
  @param len number of bytes in the payload
  */
  void callback_worker(char* topic, byte* payload, unsigned int len);
  bool update_status(bool force); //!< @brief Retrieve status from the power panel and NTP
  bool is_primary_host_online(void); //!< @brief Is the Primary Host online?

private:
  void lock_network(void);
  void unlock_network(void);
  bool set_up_schedule_dataset(dataset_t *schedule, unsigned int max_commands);
  bool copy_schedule(uint32_t outs[], uint32_t rows_count, dataset_t *schedule);
  void free_schedule(dataset_t *schedule);
  void generateNames(char dev_id);
  bool connect_to_broker();
  bool subscribeTopics();
  void disconnect_from_broker();
  void publish_births(void);
  void publish_node_data(void);
  bool process_host_state_message(char* topic, byte* payload, unsigned int len);
  bool process_node_cmd_message(char* topic, byte* payload, unsigned int len);
  uint32_t heaters_only(uint32_t heater_word);
  void reset_teensy(void);

  PowerPanel panel; //!< @brief the Power Panel
  power_panel_device_data_t dev; //!< @brief all data related to the power panel

#if defined(SIMULATED_POWER_PANEL)
  PowerPanelSimulator ppsim;    //!< @brief the simulated power panel
#endif

  // MQTT variables
  Threads::Mutex *network_lock; //!< @brief ensure single-threaded access to the network
  EthernetClient enet; //!< @brief net client for broker
  PubSubClient broker; //!< @brief MQTT client for broker
  int broker_index; //!< @brief broker number in use
  int last_broker_index; //!< @brief last broker that was connected
  uint32_t broker_retry_timeout_start; //!< @brief limits how often to retry connections
  bool was_connected; //!< @brief broker was connected on previous check

  // Sparkplug node and topic names
  String node_id;
  String nodeBirthTopic;
  String nodeDeathTopic;
  String nodeDataTopic;
  String nodeCmdTopic;

  // These variables each hold the current value of a metric
  uint8_t  bdSeq;
  bool     nodeReboot;
  bool     nodeRebirth;
  bool     nodeNextServer;
  int64_t  commsVersion;
  const char *firmwareVersion;
  uint64_t uniqueId;
  int32_t  commsChannel;
  ntp_status_t ntpStatus;
  char ntpServer[24];
  char *ntpServerPtr;
  const char *panelStatus; //!< @brief power panel comms status as a string
  dataset_t activeSchedule;
  dataset_t nextSchedule;

#if defined(SIMULATED_POWER_PANEL)
  typedef struct simulation_settings{
    bool noReply;
    bool parityError;
    bool invalidHex;
    bool checksumError;
    bool noStrobe;
    bool shortReply;
    bool longReply;
    uint32_t v_on;
    uint32_t v_off;
    uint32_t il_on;
    uint32_t il_off;
    uint32_t ir_on;
    uint32_t ir_off;
    int64_t  clock_offset;
  } simulation_settings_t;

  simulation_settings_t sim; //!< @brief settings for the simulated power panel
#endif

  metric_spec_list_t node_metrics;
  metric_t pub_metrics[MAX_SPARKPLUG_METRICS];
  uint8_t buffer[BIN_BUF_SIZE]; // buffer for writing network data
  bool publish_requested;

  // sparkplugb storage
  so_sparkplugb_t sp;
  payload_t rx_payload;

  // Primary Host tracking
  bool first_broker_connection;
  bool primary_host_online;
  uint64_t primary_host_statetime;
  uint32_t primary_host_timeout_start;
  uint32_t primary_host_timeout;
};

#endif  // __PPI_H__
