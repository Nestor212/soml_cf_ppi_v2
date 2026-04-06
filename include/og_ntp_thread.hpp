/**
 * @file ntp_thread.hpp
 * @author Adrian Loeff
 * @brief Manage interactions with the NTP server in a separate thread.
 * @version 1.0.
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 */

#ifndef NTP_THREAD_HPP
#define NTP_THREAD_HPP


#include <stdbool.h>
#include <stdint.h>
#include "IPAddress.h"
#include "TeensyThreads.h"


// Structure used to return current NTP status
typedef struct ntp_status{
  bool       synchronized;       // Are we currently synchronized to an NTP server?
  uint64_t   last_update_time;   // NTP time at last successful NTP update
  IPAddress *server_ip;          // The current NTP server IP address (NULL if none)
} ntp_status_t;


// Start the NTP update thread.  The ntp_server_ips array must remain in memory.
bool ntp_start(IPAddress *ntp_server_ips, int num_ntp_server_ips, Threads::Mutex *network_lock);

// Returns the milliseconds since Jan 1, 1970 UTC from the NTP object.
uint64_t ntp_get_current_time_millis(void);

// Returns the current status of our NTP connection.
void ntp_get_status(ntp_status_t *status);

// Set the offset of the clock in milliseconds.
void ntp_set_offset(int64_t clock_offset_ms);

// Get the currently set offset of the clock in milliseconds.
int64_t ntp_get_offset(void);


#endif  // NTP_THREAD_HPP
