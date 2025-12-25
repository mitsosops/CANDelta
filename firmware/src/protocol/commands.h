#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include <stdbool.h>

// Command opcodes (host -> device)
// NOTE: Avoid 0x02 (STX) and 0x03 (ETX) as they conflict with framing!
typedef enum {
    CMD_PING            = 0x01,
    CMD_GET_VERSION     = 0x04,
    CMD_GET_STATUS      = 0x05,
    CMD_START_CAPTURE   = 0x10,
    CMD_STOP_CAPTURE    = 0x11,
    CMD_SET_SPEED       = 0x20,
    CMD_SET_FILTER      = 0x21,
    CMD_CLEAR_FILTERS   = 0x22,
    CMD_SET_MODE        = 0x23,
    CMD_TRANSMIT_FRAME  = 0x30,
} command_opcode_t;

// Response codes (device -> host)
typedef enum {
    RSP_ACK             = 0x80,
    RSP_NAK             = 0x81,
    RSP_VERSION         = 0x82,
    RSP_STATUS          = 0x83,
    RSP_CAN_FRAME       = 0x84,  // Async CAN frame data
} response_code_t;

// Device status
typedef struct {
    uint8_t  protocol_version;
    uint8_t  mode;
    uint32_t can_speed;
    bool     capture_active;
    uint8_t  error_flags;
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
} device_status_t;

// Initialize command processor
void commands_init(void);

// Process incoming commands (call from main loop)
void commands_process(void);

// Get current device status
void commands_get_status(device_status_t *status);

#endif // COMMANDS_H
