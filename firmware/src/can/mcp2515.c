#include "mcp2515.h"
#include "candelta_config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"

// Mutex to protect SPI access between cores
static mutex_t spi_mutex;

// Statistics tracking
static mcp2515_stats_t stats = {0};

// Callbacks
static mcp2515_tx_cb_t tx_callback = NULL;
static mcp2515_rx_cb_t rx_callback = NULL;

// Stored configuration (survives reset for restore)
static struct {
    // Timing
    mcp2515_timing_t timing;
    bool timing_set;
    mcp2515_mode_t mode;

    // Filters/masks
    mcp2515_filter_t filters[6];
    bool filter_set[6];           // Track which filters are configured
    uint32_t masks[2];
    bool mask_extended[2];
    bool mask_set[2];             // Track which masks are configured
    bool filters_enabled;         // Whether filter mode is active

    // TX settings
    bool oneshot_enabled;
} saved_config = {0};

// MCP2515 SPI Commands
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

// MCP2515 Registers
#define REG_CANSTAT     0x0E
#define REG_CANCTRL     0x0F
#define REG_CNF1        0x2A
#define REG_CNF2        0x29
#define REG_CNF3        0x28
#define REG_CANINTE     0x2B
#define REG_CANINTF     0x2C
#define REG_EFLG        0x2D
#define REG_TXB0CTRL    0x30
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

// Forward declarations for static helpers
static uint8_t get_filter_reg(uint8_t filter_num);
static void write_id_registers(uint8_t base_reg, uint32_t id, bool extended);

static void spi_cs_select(void) {
    gpio_put(MCP2515_PIN_CS, 0);
}

static void spi_cs_deselect(void) {
    gpio_put(MCP2515_PIN_CS, 1);
}

static uint8_t spi_read_register(uint8_t reg) {
    uint8_t cmd[2] = {MCP_READ, reg};
    uint8_t val;

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, cmd, 2);
    spi_read_blocking(MCP2515_SPI_PORT, 0x00, &val, 1);
    spi_cs_deselect();

    return val;
}

static void spi_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[3] = {MCP_WRITE, reg, value};

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, tx, 3);
    spi_cs_deselect();
}

static void spi_modify_register(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t tx[4] = {MCP_BIT_MODIFY, reg, mask, value};

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, tx, 4);
    spi_cs_deselect();
}

void mcp2515_init(void) {
    // Initialize SPI mutex for multi-core safety
    mutex_init(&spi_mutex);

    // Initialize SPI
    spi_init(MCP2515_SPI_PORT, MCP2515_SPI_SPEED);

    gpio_set_function(MCP2515_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_PIN_SCK, GPIO_FUNC_SPI);

    // CS pin as GPIO output
    gpio_init(MCP2515_PIN_CS);
    gpio_set_dir(MCP2515_PIN_CS, GPIO_OUT);
    gpio_put(MCP2515_PIN_CS, 1);

    // INT pin as input
    gpio_init(MCP2515_PIN_INT);
    gpio_set_dir(MCP2515_PIN_INT, GPIO_IN);
    gpio_pull_up(MCP2515_PIN_INT);

    // Reset MCP2515
    mcp2515_reset();
    sleep_ms(10);

    // Set default speed
    mcp2515_set_speed(CAN_DEFAULT_SPEED);

    // Configure RX buffers to receive any message
    // 0x64 = Accept all + BUKT (rollover to RXB1 when RXB0 full)
    spi_write_register(REG_RXB0CTRL, 0x64);  // Accept all + rollover enabled
    spi_write_register(REG_RXB1CTRL, 0x60);  // Accept all

    // Enable RX interrupts
    spi_write_register(REG_CANINTE, 0x03);

    // Default to LISTEN-ONLY mode for passive monitoring
    // Use mcp2515_set_mode(NORMAL) before TX or when CANDelta is only receiver
    mcp2515_set_mode(MCP2515_MODE_LISTEN_ONLY);
}

void mcp2515_reset(void) {
    uint8_t cmd = MCP_RESET;

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_cs_deselect();
}

bool mcp2515_reset_and_restore(void) {
    mutex_enter_blocking(&spi_mutex);

    // Issue reset command
    uint8_t cmd = MCP_RESET;
    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_cs_deselect();

    sleep_ms(10);  // Wait for reset to complete

    // Verify we're in CONFIG mode (reset default)
    uint8_t status = spi_read_register(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        mutex_exit(&spi_mutex);
        return false;
    }

    // Restore timing if previously set
    if (saved_config.timing_set) {
        spi_write_register(REG_CNF1, saved_config.timing.cnf1);
        spi_write_register(REG_CNF2, saved_config.timing.cnf2);
        spi_write_register(REG_CNF3, saved_config.timing.cnf3);
    }

    // Restore filters
    for (int i = 0; i < 6; i++) {
        if (saved_config.filter_set[i]) {
            uint8_t reg = get_filter_reg(i);
            write_id_registers(reg, saved_config.filters[i].id,
                              saved_config.filters[i].extended);
        }
    }

    // Restore masks
    for (int i = 0; i < 2; i++) {
        if (saved_config.mask_set[i]) {
            uint8_t reg = (i == 0) ? REG_RXM0 : REG_RXM1;
            write_id_registers(reg, saved_config.masks[i],
                              saved_config.mask_extended[i]);
        }
    }

    // Restore filter mode or default to accept-all
    if (saved_config.filters_enabled) {
        spi_write_register(REG_RXB0CTRL, 0x04);  // Filter mode + rollover
        spi_write_register(REG_RXB1CTRL, 0x00);  // Filter mode
    } else {
        spi_write_register(REG_RXB0CTRL, 0x64);  // Accept all + rollover
        spi_write_register(REG_RXB1CTRL, 0x60);  // Accept all
    }

    // Enable RX interrupts
    spi_write_register(REG_CANINTE, 0x03);

    // Restore one-shot mode if enabled
    if (saved_config.oneshot_enabled) {
        spi_modify_register(REG_CANCTRL, CANCTRL_OSM, CANCTRL_OSM);
    }

    // Clear error flags and interrupt flags
    spi_write_register(REG_EFLG, 0x00);
    spi_write_register(REG_CANINTF, 0x00);

    mutex_exit(&spi_mutex);

    // Reset statistics
    mcp2515_reset_stats();

    // Restore mode (defaults to LISTEN_ONLY if not set)
    mcp2515_mode_t target_mode = saved_config.mode;
    if (target_mode == MCP2515_MODE_CONFIG) {
        target_mode = MCP2515_MODE_LISTEN_ONLY;  // Don't stay in config
    }
    return mcp2515_set_mode(target_mode);
}

bool mcp2515_set_mode(mcp2515_mode_t mode) {
    uint8_t mode_bits;

    switch (mode) {
        case MCP2515_MODE_NORMAL:     mode_bits = MODE_NORMAL; break;
        case MCP2515_MODE_SLEEP:      mode_bits = MODE_SLEEP; break;
        case MCP2515_MODE_LOOPBACK:   mode_bits = MODE_LOOPBACK; break;
        case MCP2515_MODE_LISTEN_ONLY: mode_bits = MODE_LISTEN; break;
        case MCP2515_MODE_CONFIG:     mode_bits = MODE_CONFIG; break;
        default: return false;
    }

    mutex_enter_blocking(&spi_mutex);

    // Clear any pending interrupt flags before mode change
    spi_write_register(REG_CANINTF, 0x00);

    // Request mode change
    spi_modify_register(REG_CANCTRL, MODE_MASK, mode_bits);

    // Wait for mode change with timeout (up to 50ms)
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        uint8_t status = spi_read_register(REG_CANSTAT);
        if ((status & MODE_MASK) == mode_bits) {
            saved_config.mode = mode;  // Save for restore after reset
            mutex_exit(&spi_mutex);
            return true;
        }
    }

    mutex_exit(&spi_mutex);
    return false;
}

bool mcp2515_set_timing(const mcp2515_timing_t *timing) {
    mutex_enter_blocking(&spi_mutex);

    // Save current mode to restore after timing change
    uint8_t prev_mode = spi_read_register(REG_CANSTAT) & MODE_MASK;

    // Try to enter CONFIG mode without reset
    spi_modify_register(REG_CANCTRL, MODE_MASK, MODE_CONFIG);

    // Wait for mode change (MCP2515 won't change if TX in progress)
    bool in_config = false;
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        uint8_t status = spi_read_register(REG_CANSTAT);
        if ((status & MODE_MASK) == MODE_CONFIG) {
            in_config = true;
            break;
        }
    }

    if (!in_config) {
        mutex_exit(&spi_mutex);
        return false;
    }

    // Write timing registers
    spi_write_register(REG_CNF1, timing->cnf1);
    spi_write_register(REG_CNF2, timing->cnf2);
    spi_write_register(REG_CNF3, timing->cnf3);

    // Save timing for restore after reset
    saved_config.timing = *timing;
    saved_config.timing_set = true;

    // Clear error flags (EFLG) - may have accumulated during wrong-speed operation
    spi_write_register(REG_EFLG, 0x00);

    // Clear interrupt flags
    spi_write_register(REG_CANINTF, 0x00);

    // Note: RXBxCTRL, filters, masks, and CANINTE all preserve their values
    // when entering CONFIG mode - only RESET clears them

    // Restore previous mode (within mutex to prevent race with Core 1)
    spi_modify_register(REG_CANCTRL, MODE_MASK, prev_mode);

    // Wait for mode change
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        uint8_t status = spi_read_register(REG_CANSTAT);
        if ((status & MODE_MASK) == prev_mode) {
            break;
        }
    }

    mutex_exit(&spi_mutex);
    return true;
}

bool mcp2515_set_speed(uint32_t speed_bps) {
    // CNF values for 16MHz crystal (from arduino-mcp2515 library)
    mcp2515_timing_t timing;

    switch (speed_bps) {
        case CAN_SPEED_125KBPS:
            timing.cnf1 = 0x03; timing.cnf2 = 0xF0; timing.cnf3 = 0x86;
            break;
        case CAN_SPEED_250KBPS:
            timing.cnf1 = 0x41; timing.cnf2 = 0xF1; timing.cnf3 = 0x85;
            break;
        case CAN_SPEED_500KBPS:
            timing.cnf1 = 0x00; timing.cnf2 = 0xF0; timing.cnf3 = 0x86;
            break;
        case CAN_SPEED_1MBPS:
            timing.cnf1 = 0x00; timing.cnf2 = 0xD0; timing.cnf3 = 0x82;
            break;
        default:
            return false;
    }

    return mcp2515_set_timing(&timing);
}

bool mcp2515_receive(can_frame_t *frame) {
    // Check interrupt pin (active low)
    if (gpio_get(MCP2515_PIN_INT)) {
        return false;  // No interrupt, no data
    }

    uint8_t status = spi_read_register(REG_CANINTF);

    if (status & 0x01) {
        // RX buffer 0 has data
        uint8_t cmd = MCP_READ_RX0;
        uint8_t buf[13];

        spi_cs_select();
        spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
        spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
        spi_cs_deselect();

        // Parse ID
        frame->extended = (buf[1] & 0x08) != 0;
        if (frame->extended) {
            frame->id = ((uint32_t)(buf[0]) << 21) |
                       ((uint32_t)(buf[1] & 0xE0) << 13) |
                       ((uint32_t)(buf[1] & 0x03) << 16) |
                       ((uint32_t)buf[2] << 8) |
                       buf[3];
        } else {
            frame->id = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);
        }

        frame->rtr = (buf[4] & 0x40) != 0;
        frame->dlc = buf[4] & 0x0F;
        if (frame->dlc > 8) frame->dlc = 8;

        for (int i = 0; i < frame->dlc; i++) {
            frame->data[i] = buf[5 + i];
        }

        // Clear interrupt flag
        spi_modify_register(REG_CANINTF, 0x01, 0x00);

        return true;
    }

    if (status & 0x02) {
        // RX buffer 1 has data
        uint8_t cmd = MCP_READ_RX1;
        uint8_t buf[13];

        spi_cs_select();
        spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
        spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
        spi_cs_deselect();

        // Parse ID
        frame->extended = (buf[1] & 0x08) != 0;
        if (frame->extended) {
            frame->id = ((uint32_t)(buf[0]) << 21) |
                       ((uint32_t)(buf[1] & 0xE0) << 13) |
                       ((uint32_t)(buf[1] & 0x03) << 16) |
                       ((uint32_t)buf[2] << 8) |
                       buf[3];
        } else {
            frame->id = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);
        }

        frame->rtr = (buf[4] & 0x40) != 0;
        frame->dlc = buf[4] & 0x0F;
        if (frame->dlc > 8) frame->dlc = 8;

        for (int i = 0; i < frame->dlc; i++) {
            frame->data[i] = buf[5 + i];
        }

        // Clear interrupt flag
        spi_modify_register(REG_CANINTF, 0x02, 0x00);

        return true;
    }

    return false;
}

// Helper to parse RX buffer data into frame
static void parse_rx_buffer(const uint8_t *buf, can_frame_t *frame) {
    frame->extended = (buf[1] & 0x08) != 0;
    if (frame->extended) {
        frame->id = ((uint32_t)(buf[0]) << 21) |
                   ((uint32_t)(buf[1] & 0xE0) << 13) |
                   ((uint32_t)(buf[1] & 0x03) << 16) |
                   ((uint32_t)buf[2] << 8) |
                   buf[3];
    } else {
        frame->id = ((uint16_t)buf[0] << 3) | (buf[1] >> 5);
    }
    frame->rtr = (buf[4] & 0x40) != 0;
    frame->dlc = buf[4] & 0x0F;
    if (frame->dlc > 8) frame->dlc = 8;
    for (int i = 0; i < frame->dlc; i++) {
        frame->data[i] = buf[5 + i];
    }
}

int mcp2515_receive_all(can_frame_t *frames, int max_frames) {
    int count = 0;

    // Loop while INT is low (frames pending) and we have buffer space
    // This prevents race condition where frames arrive while we're reading
    while (count < max_frames && !gpio_get(MCP2515_PIN_INT)) {
        // Acquire SPI mutex for multi-core safety
        mutex_enter_blocking(&spi_mutex);

        // Read CANINTF each iteration to catch newly arrived frames
        uint8_t status = spi_read_register(REG_CANINTF);

        if (status & 0x01) {
            // RXB0 has data - READ_RX0 command auto-clears RX0IF
            uint8_t cmd = MCP_READ_RX0;
            uint8_t buf[13];

            spi_cs_select();
            spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
            spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
            spi_cs_deselect();

            parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            stats.rx_frames++;
            if (rx_callback) rx_callback(&frames[count]);
            count++;
        }

        if ((status & 0x02) && count < max_frames) {
            // RXB1 has data - READ_RX1 command auto-clears RX1IF
            uint8_t cmd = MCP_READ_RX1;
            uint8_t buf[13];

            spi_cs_select();
            spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
            spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
            spi_cs_deselect();

            parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            stats.rx_frames++;
            if (rx_callback) rx_callback(&frames[count]);
            count++;
        }

        // Release SPI mutex
        mutex_exit(&spi_mutex);

        // If nothing was read but INT still low, something's wrong - break to avoid infinite loop
        if ((status & 0x03) == 0) {
            break;
        }
    }

    // No manual flag clearing needed - READ_RX commands auto-clear

    return count;
}

bool mcp2515_transmit(const can_frame_t *frame) {
    // Acquire SPI mutex for multi-core safety
    mutex_enter_blocking(&spi_mutex);

    // TX only works in NORMAL mode - check current mode
    uint8_t canstat = spi_read_register(REG_CANSTAT);
    if ((canstat & MODE_MASK) != MODE_NORMAL) {
        mutex_exit(&spi_mutex);
        stats.tx_errors++;
        if (tx_callback) tx_callback(false);
        return false;  // Not in NORMAL mode - can't transmit
    }

    // Check if TX buffer 0 is free
    uint8_t status = spi_read_register(REG_TXB0CTRL);
    if (status & 0x08) {
        mutex_exit(&spi_mutex);
        stats.tx_errors++;
        if (tx_callback) tx_callback(false);
        return false;  // TX in progress
    }

    uint8_t buf[13];

    if (frame->extended) {
        buf[0] = (frame->id >> 21) & 0xFF;
        buf[1] = ((frame->id >> 13) & 0xE0) | 0x08 | ((frame->id >> 16) & 0x03);
        buf[2] = (frame->id >> 8) & 0xFF;
        buf[3] = frame->id & 0xFF;
    } else {
        buf[0] = (frame->id >> 3) & 0xFF;
        buf[1] = (frame->id << 5) & 0xE0;
        buf[2] = 0;
        buf[3] = 0;
    }

    buf[4] = frame->dlc | (frame->rtr ? 0x40 : 0);

    for (int i = 0; i < frame->dlc && i < 8; i++) {
        buf[5 + i] = frame->data[i];
    }

    // Load TX buffer
    uint8_t cmd = MCP_LOAD_TX0;
    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_write_blocking(MCP2515_SPI_PORT, buf, 13);
    spi_cs_deselect();

    // Request to send
    cmd = MCP_RTS | 0x01;
    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_cs_deselect();

    stats.tx_frames++;
    mutex_exit(&spi_mutex);

    if (tx_callback) tx_callback(true);
    return true;
}

// Helper: get filter register base address
static uint8_t get_filter_reg(uint8_t filter_num) {
    static const uint8_t filter_regs[] = {
        REG_RXF0, REG_RXF1, REG_RXF2,
        REG_RXF3, REG_RXF4, REG_RXF5
    };
    return (filter_num < 6) ? filter_regs[filter_num] : 0;
}

// Helper: write ID to filter/mask registers
static void write_id_registers(uint8_t base_reg, uint32_t id, bool extended) {
    uint8_t buf[4];

    if (extended) {
        // 29-bit extended ID
        buf[0] = (id >> 21) & 0xFF;                           // SIDH: SID[10:3]
        buf[1] = ((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03);  // SIDL: SID[2:0] + EXIDE + EID[17:16]
        buf[2] = (id >> 8) & 0xFF;                            // EID8: EID[15:8]
        buf[3] = id & 0xFF;                                   // EID0: EID[7:0]
    } else {
        // 11-bit standard ID
        buf[0] = (id >> 3) & 0xFF;    // SIDH: SID[10:3]
        buf[1] = (id << 5) & 0xE0;    // SIDL: SID[2:0], EXIDE=0
        buf[2] = 0;                   // EID8: unused
        buf[3] = 0;                   // EID0: unused
    }

    for (int i = 0; i < 4; i++) {
        spi_write_register(base_reg + i, buf[i]);
    }
}

bool mcp2515_set_filter(uint8_t filter_num, const mcp2515_filter_t *filter) {
    if (filter_num > 5 || filter == NULL) {
        return false;
    }

    mutex_enter_blocking(&spi_mutex);

    // Must be in config mode to set filters
    uint8_t status = spi_read_register(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        mutex_exit(&spi_mutex);
        return false;
    }

    uint8_t reg = get_filter_reg(filter_num);
    write_id_registers(reg, filter->id, filter->extended);

    // Store configuration for reapplication after reset
    saved_config.filters[filter_num] = *filter;
    saved_config.filter_set[filter_num] = true;

    mutex_exit(&spi_mutex);
    return true;
}

bool mcp2515_set_mask(uint8_t mask_num, uint32_t mask, bool extended) {
    if (mask_num > 1) {
        return false;
    }

    mutex_enter_blocking(&spi_mutex);

    // Must be in config mode to set masks
    uint8_t status = spi_read_register(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        mutex_exit(&spi_mutex);
        return false;
    }

    uint8_t reg = (mask_num == 0) ? REG_RXM0 : REG_RXM1;
    write_id_registers(reg, mask, extended);

    // Store configuration for reapplication after reset
    saved_config.masks[mask_num] = mask;
    saved_config.mask_extended[mask_num] = extended;
    saved_config.mask_set[mask_num] = true;

    mutex_exit(&spi_mutex);
    return true;
}

void mcp2515_enable_filters(void) {
    mutex_enter_blocking(&spi_mutex);

    // Configure RXB0 and RXB1 to use filters (clear RXM bits)
    // RXM[1:0] = 00 means use filters and masks
    // Keep BUKT (rollover) enabled for RXB0
    spi_write_register(REG_RXB0CTRL, 0x04);  // Filter mode + rollover
    spi_write_register(REG_RXB1CTRL, 0x00);  // Filter mode

    // Mark filters as enabled for reapplication after reset
    saved_config.filters_enabled = true;

    mutex_exit(&spi_mutex);
}

void mcp2515_clear_filters(void) {
    // Save current mode to restore after
    mcp2515_mode_t prev_mode = mcp2515_get_mode();

    mcp2515_set_mode(MCP2515_MODE_CONFIG);
    spi_write_register(REG_RXB0CTRL, 0x64);  // Accept all + rollover enabled
    spi_write_register(REG_RXB1CTRL, 0x60);  // Accept all

    // Restore previous mode
    mcp2515_set_mode(prev_mode);

    // Clear stored filter configuration
    for (int i = 0; i < 6; i++) {
        saved_config.filter_set[i] = false;
    }
    for (int i = 0; i < 2; i++) {
        saved_config.mask_set[i] = false;
    }
    saved_config.filters_enabled = false;
}

uint8_t mcp2515_get_error_flags(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_EFLG);
    mutex_exit(&spi_mutex);
    return val;
}

mcp2515_error_state_t mcp2515_get_error_state(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t eflg = spi_read_register(REG_EFLG);
    mutex_exit(&spi_mutex);

    // EFLG bit interpretation:
    // Bit 5 (0x20): TXBO - Bus-off
    // Bit 4 (0x10): TXEP - TX Error Passive
    // Bit 3 (0x08): RXEP - RX Error Passive
    // Bit 2 (0x04): TXWAR - TX Warning (TEC >= 96)
    // Bit 1 (0x02): RXWAR - RX Warning (REC >= 96)
    // Bit 0 (0x01): EWARN - Error Warning (TEC or REC >= 96)

    if (eflg & 0x20) {
        return CAN_STATE_BUS_OFF;
    }
    if (eflg & 0x18) {  // TXEP or RXEP
        return CAN_STATE_ERROR_PASSIVE;
    }
    if (eflg & 0x07) {  // TXWAR, RXWAR, or EWARN
        return CAN_STATE_ERROR_WARNING;
    }
    return CAN_STATE_ERROR_ACTIVE;
}

uint8_t mcp2515_get_canintf(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_CANINTF);
    mutex_exit(&spi_mutex);
    return val;
}

uint8_t mcp2515_get_canstat(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_CANSTAT);
    mutex_exit(&spi_mutex);
    return val;
}

uint8_t mcp2515_get_cnf1(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_CNF1);
    mutex_exit(&spi_mutex);
    return val;
}

mcp2515_mode_t mcp2515_get_mode(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t status = spi_read_register(REG_CANSTAT);
    mutex_exit(&spi_mutex);
    return (mcp2515_mode_t)((status & MODE_MASK) >> 5);
}

bool mcp2515_is_tx_ready(void) {
    // TX only works in NORMAL mode
    return mcp2515_get_mode() == MCP2515_MODE_NORMAL;
}

uint8_t mcp2515_get_tec(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_TEC);
    mutex_exit(&spi_mutex);
    return val;
}

uint8_t mcp2515_get_rec(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_REC);
    mutex_exit(&spi_mutex);
    return val;
}

void mcp2515_set_oneshot_mode(bool enabled) {
    mutex_enter_blocking(&spi_mutex);
    spi_modify_register(REG_CANCTRL, CANCTRL_OSM, enabled ? CANCTRL_OSM : 0);
    saved_config.oneshot_enabled = enabled;
    mutex_exit(&spi_mutex);
}

void mcp2515_get_stats(mcp2515_stats_t *out) {
    mutex_enter_blocking(&spi_mutex);

    // Copy current stats
    out->rx_frames = stats.rx_frames;
    out->tx_frames = stats.tx_frames;
    out->tx_errors = stats.tx_errors;

    // Read overflow/error flags from EFLG register
    uint8_t eflg = spi_read_register(REG_EFLG);
    out->rx_overflows = (eflg & 0xC0) ? 1 : 0;  // RX0OVR or RX1OVR
    out->bus_errors = (eflg & 0x3F) ? 1 : 0;    // Any error flag

    mutex_exit(&spi_mutex);
}

void mcp2515_reset_stats(void) {
    mutex_enter_blocking(&spi_mutex);
    stats.rx_frames = 0;
    stats.tx_frames = 0;
    stats.tx_errors = 0;
    stats.rx_overflows = 0;
    stats.bus_errors = 0;
    mutex_exit(&spi_mutex);
}

void mcp2515_set_tx_callback(mcp2515_tx_cb_t callback) {
    tx_callback = callback;
}

void mcp2515_set_rx_callback(mcp2515_rx_cb_t callback) {
    rx_callback = callback;
}
