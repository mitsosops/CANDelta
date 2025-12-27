#ifndef USB_COMM_H
#define USB_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "can/mcp2515.h"

// Performance statistics
typedef struct {
    uint32_t frames_per_second;   // Current FPS (updated every second)
    uint32_t peak_fps;            // Maximum FPS observed
    uint32_t dropped_frames;      // Frames lost due to buffer full
    uint8_t  buffer_utilization;  // Buffer fill percentage (0-100)
} perf_stats_t;

// Initialize USB communication
void usb_comm_init(void);

// Queue a CAN frame for transmission to host
// Thread-safe, can be called from Core 1
bool usb_queue_frame(const can_frame_t *frame);

// Transmit all queued frames to host
// Should be called from Core 0 main loop
void usb_transmit_queued(void);

// Read incoming data from USB
// Returns number of bytes read
int usb_read(uint8_t *buffer, int max_len);

// Write data to USB
void usb_write(const uint8_t *data, int len);

// Check if USB is connected
bool usb_is_connected(void);

// Get RX frame count (queued by Core 1)
uint32_t usb_get_rx_count(void);

// Get TX to host count (sent by Core 0)
uint32_t usb_get_tx_to_host_count(void);

// Debug: buffer state
uint32_t usb_get_buffer_head(void);
uint32_t usb_get_buffer_tail(void);

// Performance monitoring
void usb_get_perf_stats(perf_stats_t *stats);
void usb_update_perf_stats(void);  // Call from main loop
void usb_reset_stats(void);        // Reset all counters (call on START_CAPTURE)

#endif // USB_COMM_H
