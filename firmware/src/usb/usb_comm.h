#ifndef USB_COMM_H
#define USB_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "can/mcp2515.h"

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
uint16_t usb_get_buffer_head(void);
uint16_t usb_get_buffer_tail(void);

#endif // USB_COMM_H
