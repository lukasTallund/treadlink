#pragma once

#include "esp_err.h"
#include "data_bridge.h"

// Receives treadmill telemetry over UART from a companion ESP32 (the
// ESPHome-Treadmill-FTMS bridge), as an alternative to BLE FTMS central.
// Frame format (sender -> us), one line per update:
//
//   [TM:<speed_001kmh>:<incline_01pct>:<distance_m>:<flags>]\n
//
//   speed_001kmh   unsigned, 5 digits zero-padded, 0.01 km/h
//   incline_01pct  signed,   4 chars incl. sign, zero-padded, 0.1 %
//   distance_m     unsigned, 5 digits zero-padded, meters (total)
//   flags          2 hex digits: bit0=has_incline, bit1=has_distance
//
// Example: [TM:00850:-020:00152:03]
//
// Receive-only: there is no command channel back to the sender.

typedef void (*uart_bridge_data_cb_t)(const ftms_treadmill_data_t *data);

esp_err_t uart_bridge_init(uart_bridge_data_cb_t data_cb);

// True if a valid frame arrived within the last few seconds.
bool uart_bridge_is_active(void);
