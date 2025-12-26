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

// Custom bit timing configuration
typedef struct {
    uint8_t cnf1;  // SJW + BRP
    uint8_t cnf2;  // BTLMODE + SAM + PHSEG1 + PRSEG
    uint8_t cnf3;  // WAKFIL + PHSEG2
} mcp2515_timing_t;

// CAN statistics (centralized tracking)
typedef struct {
    uint32_t rx_frames;      // Total frames received
    uint32_t tx_frames;      // Total frames transmitted
    uint32_t tx_errors;      // TX failures (buffer busy)
    uint32_t rx_overflows;   // RX buffer overflows (from MCP2515 EFLG)
    uint32_t bus_errors;     // CAN bus errors detected
} mcp2515_stats_t;

// CAN error state (from EFLG register interpretation)
typedef enum {
    CAN_STATE_ERROR_ACTIVE,   // Normal operation (TEC/REC < 96)
    CAN_STATE_ERROR_WARNING,  // Warning threshold (TEC or REC >= 96)
    CAN_STATE_ERROR_PASSIVE,  // Error passive (TEC or REC >= 128)
    CAN_STATE_BUS_OFF         // Bus-off (TEC >= 256)
} mcp2515_error_state_t;

// Acceptance filter configuration
// MCP2515 has 6 filters and 2 masks:
//   RXB0 uses filters 0-1 with mask 0
//   RXB1 uses filters 2-5 with mask 1
typedef struct {
    uint32_t id;        // Filter ID (11-bit or 29-bit)
    uint32_t mask;      // Mask (1 = must match, 0 = don't care)
    bool     extended;  // True for 29-bit filter
} mcp2515_filter_t;

// Callback types
typedef void (*mcp2515_tx_cb_t)(bool success);           // TX completion callback
typedef void (*mcp2515_rx_cb_t)(const can_frame_t *frame); // RX frame callback

// Initialize MCP2515
void mcp2515_init(void);

// Reset MCP2515
void mcp2515_reset(void);

// Set operating mode
bool mcp2515_set_mode(mcp2515_mode_t mode);

// Get current operating mode
mcp2515_mode_t mcp2515_get_mode(void);

// Set CAN bus speed
bool mcp2515_set_speed(uint32_t speed_bps);

// Set custom bit timing (for non-standard baud rates)
bool mcp2515_set_timing(const mcp2515_timing_t *timing);

// Receive a CAN frame (non-blocking)
// Returns true if a frame was received
bool mcp2515_receive(can_frame_t *frame);

// Receive all available frames (for interrupt-driven capture)
// Returns number of frames received (0-2)
int mcp2515_receive_all(can_frame_t *frames, int max_frames);

// Transmit a CAN frame
// Returns true if frame was queued for transmission
bool mcp2515_transmit(const can_frame_t *frame);

// Set acceptance filter (filter_num: 0-5)
// Filters 0-1 apply to RXB0 (with mask 0), filters 2-5 apply to RXB1 (with mask 1)
bool mcp2515_set_filter(uint8_t filter_num, const mcp2515_filter_t *filter);

// Set acceptance mask (mask_num: 0-1)
// Mask 0 applies to RXB0/filters 0-1, Mask 1 applies to RXB1/filters 2-5
bool mcp2515_set_mask(uint8_t mask_num, uint32_t mask, bool extended);

// Enable filtering mode (must call after setting filters)
void mcp2515_enable_filters(void);

// Clear all filters (accept all)
void mcp2515_clear_filters(void);

// Get error status
uint8_t mcp2515_get_error_flags(void);

// Get interpreted error state (from EFLG register)
mcp2515_error_state_t mcp2515_get_error_state(void);

// Check if MCP2515 is ready for TX (in NORMAL mode)
bool mcp2515_is_tx_ready(void);

// Debug registers
uint8_t mcp2515_get_canintf(void);
uint8_t mcp2515_get_canstat(void);
uint8_t mcp2515_get_cnf1(void);

// Error counters (CAN 2.0B)
uint8_t mcp2515_get_tec(void);  // Transmit Error Counter
uint8_t mcp2515_get_rec(void);  // Receive Error Counter

// One-shot TX mode (no automatic retransmission on error)
void mcp2515_set_oneshot_mode(bool enabled);

// Statistics
void mcp2515_get_stats(mcp2515_stats_t *stats);
void mcp2515_reset_stats(void);

// Callbacks
void mcp2515_set_tx_callback(mcp2515_tx_cb_t callback);
void mcp2515_set_rx_callback(mcp2515_rx_cb_t callback);

#endif // MCP2515_H
