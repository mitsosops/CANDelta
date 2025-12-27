#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// MCP2515 has 6 filters and 2 masks
#define CAN_NUM_FILTERS 6
#define CAN_NUM_MASKS   2

// Filter to RX buffer mapping:
//   Filters 0-1 -> RXB0 (uses mask 0)
//   Filters 2-5 -> RXB1 (uses mask 1)
#define CAN_FILTER_RXB0_START 0
#define CAN_FILTER_RXB0_END   1
#define CAN_FILTER_RXB1_START 2
#define CAN_FILTER_RXB1_END   5

// Operating modes (matches MCP2515 CANSTAT[7:5])
typedef enum {
    CAN_MODE_NORMAL      = 0x00,
    CAN_MODE_SLEEP       = 0x01,
    CAN_MODE_LOOPBACK    = 0x02,
    CAN_MODE_LISTEN_ONLY = 0x03,
    CAN_MODE_CONFIG      = 0x04
} can_mode_t;

// Bit timing configuration (CNF registers)
typedef struct {
    uint8_t cnf1;  // SJW[7:6] + BRP[5:0]
    uint8_t cnf2;  // BTLMODE[7] + SAM[6] + PHSEG1[5:3] + PRSEG[2:0]
    uint8_t cnf3;  // SOF[7] + WAKFIL[6] + PHSEG2[2:0]
} can_timing_t;

// Filter configuration (per filter)
typedef struct {
    uint32_t id;       // Filter ID (11-bit or 29-bit)
    bool extended;     // True for 29-bit extended ID
    bool enabled;      // True if this filter is configured
} can_filter_t;

// Mask configuration (per mask)
typedef struct {
    uint32_t mask;     // Mask value (1 = must match, 0 = don't care)
    bool extended;     // True for 29-bit extended mask
    bool enabled;      // True if this mask is configured
} can_mask_t;

// Complete CAN configuration (intended settings)
typedef struct {
    // Speed/Timing
    uint32_t speed_bps;        // Speed in bps (0 if custom timing used)
    can_timing_t timing;       // CNF1/CNF2/CNF3 values
    bool custom_timing;        // True if SET_TIMING was used instead of SET_SPEED

    // Mode
    can_mode_t mode;           // Operating mode

    // Filters (6 total)
    can_filter_t filters[CAN_NUM_FILTERS];

    // Masks (2 total)
    can_mask_t masks[CAN_NUM_MASKS];

    // Filter settings
    bool filters_active;       // True = filtering enabled, False = accept all
    bool rollover_enabled;     // True = RXB0 overflow rolls to RXB1 (BUKT bit)

    // TX settings
    bool oneshot_enabled;      // One-shot TX mode (no auto-retry)

    // Runtime state
    bool capture_active;       // Capture in progress
} can_config_t;

// Actual register values (hardware state for verification)
typedef struct {
    // Timing registers
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;

    // Control/Status
    uint8_t canstat;           // OPMOD[7:5], ICOD[3:1]
    uint8_t canctrl;           // REQOP[7:5], ABAT[4], OSM[3], CLKEN[2], CLKPRE[1:0]

    // Error state
    uint8_t eflg;              // RX1OVR, RX0OVR, TXBO, TXEP, RXEP, TXWAR, RXWAR, EWARN
    uint8_t canintf;           // MERRF, WAKIF, ERRIF, TX2IF, TX1IF, TX0IF, RX1IF, RX0IF
    uint8_t tec;               // Transmit Error Counter (0-255)
    uint8_t rec;               // Receive Error Counter (0-255)

    // TX buffer control
    uint8_t txb0ctrl;          // ABTF, MLOA, TXERR, TXREQ, TXP[1:0]
    uint8_t txb1ctrl;
    uint8_t txb2ctrl;

    // RX buffer control
    uint8_t rxb0ctrl;          // RXM[6:5], RXRTR[3], BUKT[2], BUKT1[1], FILHIT0[0]
    uint8_t rxb1ctrl;          // RXM[6:5], RXRTR[3], FILHIT[2:0]
} can_registers_t;

// Config/register comparison result
typedef struct {
    bool mode_mismatch;        // Mode differs from intended
    bool timing_mismatch;      // CNF registers differ
    bool oneshot_mismatch;     // OSM bit differs
    bool filter_mode_mismatch; // RXM bits differ from filters_active
} can_config_status_t;

// Get the RX buffer index for a filter number
static inline uint8_t can_filter_get_rxb(uint8_t filter_num) {
    return (filter_num <= CAN_FILTER_RXB0_END) ? 0 : 1;
}

// Get the mask index for a filter number
static inline uint8_t can_filter_get_mask(uint8_t filter_num) {
    return (filter_num <= CAN_FILTER_RXB0_END) ? 0 : 1;
}

#endif // CAN_CONFIG_H
