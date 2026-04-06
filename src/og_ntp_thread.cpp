/**
 * @file ntp_thread.cpp
 * @author Adrian Loeff
 * @brief Manage interactions with the NTP server in a separate thread.
 * @version 1.0.
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 */


#include "ntp_thread.hpp"

#include <NativeEthernet.h>
#include <NTPClient_Generic.h>
#include <Arduino.h>


#define NTP_UPDATE_PERIOD_MS     1000 // Time between calls to NTP update()
#define NTP_UPDATE_INTERVAL_MS  60000 // How often to actually ping the server
#define NTP_TIMEOUT_INTERVAL_MS 90000 // Switch servers if no updates in this time (plus interval)


// Module variables
IPAddress *m_ntp_server_ip = NULL;  // Pointer to array of NTP servers
int m_num_ntp_server_ips = 0;       // Number of NTP servers
int m_ntp_server_idx = 0;           // Which NTP server is currently in use
ntp_status_t m_status_snapshot = {  // Snapshot copy of the NTP status
  .synchronized     = false,
  .last_update_time = 0,
  .server_ip        = NULL,
};

EthernetUDP m_ntpUDP;

NTPClient *m_ntp = NULL;    // The main NTP object

// Ensure single-threaded access to the NTP object
Threads::Mutex m_ntp_lock;
NTPClient m_ntp_snapshot;           // Snapshot copy of the NTP object
bool m_ntp_snapshot_ready = false;  // Whether the snapshot copy is valid yet

int64_t m_clock_offset_ms = 0;


// Local function prototypes
void ntpThread(void);


// Set up the NTP client and start the update thread.
bool ntp_start(IPAddress *ntp_server_ips, int num_ntp_server_ips, Threads::Mutex *network_lock){
  // Record the server IP addresses
  if(ntp_server_ips == NULL || num_ntp_server_ips <= 0)
    return false;
  m_ntp_server_ip = ntp_server_ips;
  m_num_ntp_server_ips = num_ntp_server_ips;

  // Create the NTP client pointing to the first server
  m_ntp = new NTPClient(m_ntpUDP, m_ntp_server_ip[m_ntp_server_idx], 0, NTP_UPDATE_INTERVAL_MS, network_lock);
  if(m_ntp == NULL)
    return false;

  m_ntp->begin();

  // Start the thread to update NTP periodically
  threads.addThread(ntpThread);

  // Success
  return true;
}


// Thread function for updating NTP periodically.  This function never returns.
void ntpThread(void){
  ntp_status_t status = {     // Current NTP status
    .synchronized     = false,
    .last_update_time = 0,
    .server_ip        = NULL,
  };
  unsigned long last_update  = 0;  // millis() at last successful NTP update

  if(m_ntp_server_ip != NULL)
    status.server_ip = &m_ntp_server_ip[m_ntp_server_idx];

  while(true){
    // update() only actually updates as often as set by NTP_UPDATE_INTERVAL_MS,
    // and returns false if it's too early to update or the update failed
    if(m_ntp->update()){
      // Set the system clock time returned by now()
      setTime(m_ntp->getUTCEpochTime());
      status.last_update_time = m_ntp->getUTCEpochMillis();
      last_update = millis();
      status.synchronized = true;
    }
    else{
      // Switch NTP servers if we can't update with the current server.  Note
      // this calculation is correct even when millis() wraps around to zero.
      unsigned long elapsed = millis() - last_update;
      if(elapsed >= NTP_UPDATE_INTERVAL_MS + NTP_TIMEOUT_INTERVAL_MS){
        m_ntp->end();
        m_ntp_server_idx++;
        if(m_ntp_server_idx >= m_num_ntp_server_ips)
          m_ntp_server_idx = 0;
        m_ntp->setPoolServerIP(m_ntp_server_ip[m_ntp_server_idx]);
        m_ntp->begin();
        status.synchronized = false;  // We're no longer synchronized
        if(m_ntp_server_ip != NULL)
          status.server_ip = &m_ntp_server_ip[m_ntp_server_idx];
        last_update = millis();  // Reset the timer
      }
    }

    m_ntp_lock.lock();
      // Copy the status object to its snapshot for access by other threads
      memcpy(&m_status_snapshot, &status, sizeof(m_status_snapshot));

      // Copy the NTP object to its snapshot for access by other threads
      m_ntp_snapshot = *m_ntp;
      m_ntp_snapshot_ready = true;
    m_ntp_lock.unlock();

    // Call NTP update occasionally
    threads.delay(NTP_UPDATE_PERIOD_MS);
  }

  // No return
}


// Returns the milliseconds since Jan 1, 1970 UTC from the NTP object.
uint64_t ntp_get_current_time_millis(void){
  uint64_t ntp_millis = 0;
  if(!m_ntp_snapshot_ready)
    // The NTP object hasn't been created yet
    ntp_millis = millis();
  else{
    m_ntp_lock.lock();
      ntp_millis = m_ntp_snapshot.getUTCEpochMillis();
    m_ntp_lock.unlock();
  }

  // Add the specified offset, if any
  ntp_millis += m_clock_offset_ms;

  return ntp_millis;
}


// Returns the current status of our NTP connection.
void ntp_get_status(ntp_status_t *status){
  if(status == NULL)
    // Invalid pointer - nothing to do
    return;

  // Copy the current status snapshot to the caller's object
  m_ntp_lock.lock();
    memcpy(status, &m_status_snapshot, sizeof(*status));
  m_ntp_lock.unlock();
}


// Set the offset of the clock in milliseconds.
void ntp_set_offset(int64_t clock_offset_ms){
  m_clock_offset_ms = clock_offset_ms;
}


// Get the currently set offset of the clock in milliseconds.
int64_t ntp_get_offset(void){
  return m_clock_offset_ms;
}
