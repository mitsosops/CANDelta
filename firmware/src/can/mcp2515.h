#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>

// CAN frame structure
typedef struct {
    uint32_t id;            // CAN ID (11-bit standard or 29-bit extended)
    uint8_t  dlc;           // Data length code (0-8)
    uint8_t  data[8];       // Frame data
    bool     extended;      // Extended ID flag
    bool     rtr;           // Remote transmission request
    uint64_t timestamp_us;  // Capture timestamp (microseconds)
} can_frame_t;

// MCP2515 operating modes
typedef enum {
    MCP2515_MODE_NORMAL,
    MCP2515_MODE_SLEEP,
    MCP2515_MODE_LOOPBACK,
    MCP2515_MODE_LISTEN_ONLY,
    MCP2515_MODE_CONFIG
} mcp2515_mode_t;

// Initialize MCP2515
void mcp2515_init(void);

// Reset MCP2515
void mcp2515_reset(void);

// Set operating mode
bool mcp2515_set_mode(mcp2515_mode_t mode);

// Set CAN bus speed
bool mcp2515_set_speed(uint32_t speed_bps);

// Receive a CAN frame (non-blocking)
// Returns true if a frame was received
bool mcp2515_receive(can_frame_t *frame);

// Receive all available frames (for interrupt-driven capture)
// Returns number of frames received (0-2)
int mcp2515_receive_all(can_frame_t *frames, int max_frames);

// Transmit a CAN frame
// Returns true if frame was queued for transmission
bool mcp2515_transmit(const can_frame_t *frame);

// Set acceptance filter
void mcp2515_set_filter(uint8_t filter_num, uint32_t id, uint32_t mask, bool extended);

// Clear all filters (accept all)
void mcp2515_clear_filters(void);

// Get error status
uint8_t mcp2515_get_error_flags(void);

// Check if MCP2515 is ready
bool mcp2515_is_ready(void);

// Debug registers
uint8_t mcp2515_get_canintf(void);
uint8_t mcp2515_get_canstat(void);
uint8_t mcp2515_get_cnf1(void);

#endif // MCP2515_H
