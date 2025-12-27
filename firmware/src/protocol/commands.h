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
    CMD_DEBUG           = 0x06,
    CMD_GET_PERF_STATS  = 0x07,
    CMD_GET_DEVICE_ID   = 0x08,  // Get unique device ID
    CMD_GET_ERROR_COUNTERS = 0x09,  // Get TEC/REC/ErrorState
    CMD_LIST_COMMANDS   = 0x0A,  // List available commands
    CMD_GET_CONFIG      = 0x0B,  // Get full configuration
    CMD_GET_REGISTERS   = 0x0C,  // Get raw register values
    CMD_START_CAPTURE   = 0x10,
    CMD_STOP_CAPTURE    = 0x11,
    CMD_SET_SPEED       = 0x20,
    CMD_SET_FILTER      = 0x21,
    CMD_CLEAR_FILTERS   = 0x22,
    CMD_SET_MODE        = 0x23,
    CMD_SET_TIMING      = 0x24,  // Custom bit timing (CNF1/2/3)
    CMD_SET_MASK        = 0x25,  // Set acceptance mask
    CMD_SET_ONESHOT     = 0x26,  // Enable/disable one-shot TX mode
    CMD_RESET_CAN       = 0x27,  // Reset CAN controller and restore config (clears TEC/REC)
    CMD_TRANSMIT_FRAME  = 0x30,
} command_opcode_t;

// Response codes (device -> host)
typedef enum {
    RSP_ACK             = 0x80,
    RSP_NAK             = 0x81,
    RSP_VERSION         = 0x82,
    RSP_STATUS          = 0x83,
    RSP_CAN_FRAME       = 0x84,  // Async CAN frame data
    RSP_DEBUG           = 0x85,
    RSP_PERF_STATS      = 0x86,
    RSP_DEVICE_ID       = 0x87,  // Unique device ID (8 bytes)
    RSP_ERROR_COUNTERS  = 0x88,  // TEC + REC + ErrorState
    RSP_COMMAND_LIST    = 0x89,  // List of available commands
    RSP_CONFIG          = 0x8A,  // Full configuration
    RSP_REGISTERS       = 0x8B,  // Raw register values
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
