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

#endif // USB_COMM_H
