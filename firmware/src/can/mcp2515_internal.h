#ifndef MCP2515_INTERNAL_H
#define MCP2515_INTERNAL_H

#include "mcp2515.h"
#include "can_config.h"
#include "candelta_config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"

// ============================================================================
// MCP2515 SPI Commands
// ============================================================================

#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_RTS         0x80
#define MCP_READ_STATUS 0xA0
#define MCP_RX_STATUS   0xB0
#define MCP_BIT_MODIFY  0x05
#define MCP_READ_RX0    0x90
#define MCP_READ_RX1    0x94
#define MCP_LOAD_TX0    0x40
#define MCP_LOAD_TX1    0x42
#define MCP_LOAD_TX2    0x44

// ============================================================================
// MCP2515 Registers
// ============================================================================

#define REG_CANSTAT     0x0E
#define REG_CANCTRL     0x0F
#define REG_CNF1        0x2A
#define REG_CNF2        0x29
#define REG_CNF3        0x28
#define REG_CANINTE     0x2B
#define REG_CANINTF     0x2C
#define REG_EFLG        0x2D
#define REG_TXB0CTRL    0x30
#define REG_TXB1CTRL    0x40
#define REG_TXB2CTRL    0x50
#define REG_RXB0CTRL    0x60
#define REG_RXB1CTRL    0x70
#define REG_TEC         0x1C  // Transmit Error Counter
#define REG_REC         0x1D  // Receive Error Counter

// Filter registers (each is 4 bytes: SIDH, SIDL, EID8, EID0)
#define REG_RXF0        0x00
#define REG_RXF1        0x04
#define REG_RXF2        0x08
#define REG_RXF3        0x10  // Note: not sequential!
#define REG_RXF4        0x14
#define REG_RXF5        0x18

// Mask registers (each is 4 bytes: SIDH, SIDL, EID8, EID0)
#define REG_RXM0        0x20
#define REG_RXM1        0x24

// Mode bits in CANCTRL
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTEN     0x60
#define MODE_CONFIG     0x80
#define MODE_MASK       0xE0
#define CANCTRL_OSM     0x08  // One-Shot Mode bit

// CANINTE interrupt enable bits
#define CANINTE_RX0IE   0x01
#define CANINTE_RX1IE   0x02
#define CANINTE_TX0IE   0x04
#define CANINTE_TX1IE   0x08
#define CANINTE_TX2IE   0x10
#define CANINTE_ERRIE   0x20
#define CANINTE_WAKIE   0x40
#define CANINTE_MERRE   0x80

// CANINTF interrupt flag bits
#define CANINTF_RX0IF   0x01
#define CANINTF_RX1IF   0x02
#define CANINTF_TX0IF   0x04
#define CANINTF_TX1IF   0x08
#define CANINTF_TX2IF   0x10
#define CANINTF_ERRIF   0x20
#define CANINTF_WAKIF   0x40
#define CANINTF_MERRF   0x80

// ============================================================================
// Shared state (defined in mcp2515_common.c)
// ============================================================================

extern mutex_t mcp2515_spi_mutex;
extern mcp2515_stats_t mcp2515_stats;
extern can_config_t mcp2515_config;
extern mcp2515_tx_cb_t mcp2515_tx_callback;
extern mcp2515_rx_cb_t mcp2515_rx_callback;
extern mcp2515_error_cb_t mcp2515_error_callback;

// ============================================================================
// SPI mutex wrappers
// Same for both polling and IRQ modes - ISR is signal-only and doesn't use SPI
// ============================================================================

static inline void mcp2515_spi_lock(void) {
    mutex_enter_blocking(&mcp2515_spi_mutex);
}

static inline void mcp2515_spi_unlock(void) {
    mutex_exit(&mcp2515_spi_mutex);
}

// ============================================================================
// Internal SPI helper functions (defined in mcp2515_common.c)
// ============================================================================

static inline void spi_cs_select(void) {
    gpio_put(MCP2515_PIN_CS, 0);
}

static inline void spi_cs_deselect(void) {
    gpio_put(MCP2515_PIN_CS, 1);
}

// Read a single register (no mutex - caller must hold mutex)
static inline uint8_t spi_read_register_raw(uint8_t reg) {
    uint8_t cmd[2] = {MCP_READ, reg};
    uint8_t val;

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, cmd, 2);
    spi_read_blocking(MCP2515_SPI_PORT, 0x00, &val, 1);
    spi_cs_deselect();

    return val;
}

// Write a single register (no mutex - caller must hold mutex)
static inline void spi_write_register_raw(uint8_t reg, uint8_t value) {
    uint8_t tx[3] = {MCP_WRITE, reg, value};

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, tx, 3);
    spi_cs_deselect();
}

// Bit modify a register (no mutex - caller must hold mutex)
static inline void spi_modify_register_raw(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t tx[4] = {MCP_BIT_MODIFY, reg, mask, value};

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, tx, 4);
    spi_cs_deselect();
}

// READ STATUS command - fast 2-byte read of RX/TX status (no mutex - caller must hold mutex)
// Returns: bit0=RX0IF, bit1=RX1IF, bit2=TX0REQ, bit3=TX0IF, bit4=TX1REQ, bit5=TX1IF, bit6=TX2REQ, bit7=TX2IF
static inline uint8_t spi_read_status_raw(void) {
    uint8_t cmd = MCP_READ_STATUS;
    uint8_t val;

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_read_blocking(MCP2515_SPI_PORT, 0x00, &val, 1);
    spi_cs_deselect();

    return val;
}

// ============================================================================
// Internal helper functions (defined in mcp2515_common.c)
// ============================================================================

// Get filter register base address
uint8_t mcp2515_get_filter_reg(uint8_t filter_num);

// Write ID to filter/mask registers (must be in config mode, no mutex)
void mcp2515_write_id_registers(uint8_t base_reg, uint32_t id, bool extended);

// Parse RX buffer data into frame structure
void mcp2515_parse_rx_buffer(const uint8_t *buf, can_frame_t *frame);

// ============================================================================
// Shared RX buffer read helper
// ============================================================================

// Read RX buffer using fast READ_RX0/RX1 commands (auto-clears flags)
// IMPORTANT: Caller must hold SPI lock (mcp2515_spi_lock or spinlock in IRQ)
// cmd: MCP_READ_RX0 (0x90) or MCP_READ_RX1 (0x94)
// buf: Output buffer, 13 bytes [SIDH, SIDL, EID8, EID0, DLC, D0-D7]
static inline void mcp2515_read_rxb(uint8_t cmd, uint8_t buf[13]) {
    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
    spi_cs_deselect();
}

// ============================================================================
// Shared RX drain loop (used by both polling and IRQ modes)
// ============================================================================

// Drain RX buffers based on READ_STATUS flags. Caller must hold SPI mutex.
// Does NOT check INT pin - callers should do fast INT check before locking.
// Returns number of frames read.
static inline int mcp2515_drain_rx_locked(can_frame_t *frames, int max_frames) {
    int count = 0;
    uint8_t buf[13];

    while (count < max_frames) {
        uint8_t status = spi_read_status_raw();
        uint8_t rx_flags = status & 0x03;

        if (rx_flags == 0) break;  // No more frames pending

        // Read RXB0 if has data
        if (rx_flags & 0x01) {
            mcp2515_read_rxb(MCP_READ_RX0, buf);
            mcp2515_parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            mcp2515_stats.rx_frames++;
            if (mcp2515_rx_callback) mcp2515_rx_callback(&frames[count]);
            count++;
            if (count >= max_frames) break;
        }

        // Read RXB1 if has data
        if (count < max_frames && (rx_flags & 0x02)) {
            mcp2515_read_rxb(MCP_READ_RX1, buf);
            mcp2515_parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            mcp2515_stats.rx_frames++;
            if (mcp2515_rx_callback) mcp2515_rx_callback(&frames[count]);
            count++;
        }
    }

    return count;
}

#endif // MCP2515_INTERNAL_H
