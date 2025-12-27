#include "mcp2515_internal.h"

// ============================================================================
// Shared state
// ============================================================================

mutex_t mcp2515_spi_mutex;
mcp2515_stats_t mcp2515_stats = {0};
can_config_t mcp2515_config = {0};
mcp2515_tx_cb_t mcp2515_tx_callback = NULL;
mcp2515_rx_cb_t mcp2515_rx_callback = NULL;
mcp2515_error_cb_t mcp2515_error_callback = NULL;

// ============================================================================
// Internal helper functions
// ============================================================================

uint8_t mcp2515_get_filter_reg(uint8_t filter_num) {
    static const uint8_t filter_regs[] = {
        REG_RXF0, REG_RXF1, REG_RXF2,
        REG_RXF3, REG_RXF4, REG_RXF5
    };
    return (filter_num < 6) ? filter_regs[filter_num] : 0;
}

void mcp2515_write_id_registers(uint8_t base_reg, uint32_t id, bool extended) {
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
        spi_write_register_raw(base_reg + i, buf[i]);
    }
}

void mcp2515_parse_rx_buffer(const uint8_t *buf, can_frame_t *frame) {
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

int mcp2515_drain_rx_locked(can_frame_t *frames, int max_frames) {
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

// ============================================================================
// Initialization
// ============================================================================

void mcp2515_init(void) {
    // Initialize SPI mutex for multi-core safety
    mutex_init(&mcp2515_spi_mutex);

    // Initialize config with defaults
    mcp2515_config.speed_bps = CAN_DEFAULT_SPEED;
    mcp2515_config.custom_timing = false;
    mcp2515_config.mode = CAN_MODE_LISTEN_ONLY;
    mcp2515_config.filters_active = false;
    mcp2515_config.rollover_enabled = true;  // Enable rollover by default
    mcp2515_config.oneshot_enabled = false;
    mcp2515_config.capture_active = false;

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
    spi_write_register_raw(REG_RXB0CTRL, 0x64);  // Accept all + rollover enabled
    spi_write_register_raw(REG_RXB1CTRL, 0x60);  // Accept all

    // Enable RX + Error interrupts (for bus-off auto-recovery)
    spi_write_register_raw(REG_CANINTE, 0x23);  // RX0IE + RX1IE + ERRIE

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
    // Issue reset command
    mcp2515_spi_lock();
    uint8_t cmd = MCP_RESET;
    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_cs_deselect();
    mcp2515_spi_unlock();

    // Wait for reset to complete - release mutex to allow Core1 to drain
    sleep_ms(10);

    // Verify we're in CONFIG mode (reset default)
    mcp2515_spi_lock();
    uint8_t status = spi_read_register_raw(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        mcp2515_spi_unlock();
        return false;
    }

    // Restore timing if previously set
    if (mcp2515_config.speed_bps != 0 || mcp2515_config.custom_timing) {
        spi_write_register_raw(REG_CNF1, mcp2515_config.timing.cnf1);
        spi_write_register_raw(REG_CNF2, mcp2515_config.timing.cnf2);
        spi_write_register_raw(REG_CNF3, mcp2515_config.timing.cnf3);
    }

    // Restore filters
    for (int i = 0; i < CAN_NUM_FILTERS; i++) {
        if (mcp2515_config.filters[i].enabled) {
            uint8_t reg = mcp2515_get_filter_reg(i);
            mcp2515_write_id_registers(reg, mcp2515_config.filters[i].id,
                              mcp2515_config.filters[i].extended);
        }
    }

    // Restore masks
    for (int i = 0; i < CAN_NUM_MASKS; i++) {
        if (mcp2515_config.masks[i].enabled) {
            uint8_t reg = (i == 0) ? REG_RXM0 : REG_RXM1;
            mcp2515_write_id_registers(reg, mcp2515_config.masks[i].mask,
                              mcp2515_config.masks[i].extended);
        }
    }

    // Restore filter mode or default to accept-all
    // RXB0CTRL: RXM[6:5]=00 for filter mode, 11 for accept all; BUKT[2] for rollover
    if (mcp2515_config.filters_active) {
        uint8_t rxb0 = mcp2515_config.rollover_enabled ? 0x04 : 0x00;  // Filter mode +/- rollover
        spi_write_register_raw(REG_RXB0CTRL, rxb0);
        spi_write_register_raw(REG_RXB1CTRL, 0x00);  // Filter mode
    } else {
        uint8_t rxb0 = mcp2515_config.rollover_enabled ? 0x64 : 0x60;  // Accept all +/- rollover
        spi_write_register_raw(REG_RXB0CTRL, rxb0);
        spi_write_register_raw(REG_RXB1CTRL, 0x60);  // Accept all
    }

    // Enable RX + Error interrupts (for bus-off auto-recovery)
    spi_write_register_raw(REG_CANINTE, 0x23);  // RX0IE + RX1IE + ERRIE

    // Restore one-shot mode if enabled
    if (mcp2515_config.oneshot_enabled) {
        spi_modify_register_raw(REG_CANCTRL, CANCTRL_OSM, CANCTRL_OSM);
    }

    // Clear error flags and interrupt flags
    spi_write_register_raw(REG_EFLG, 0x00);
    spi_write_register_raw(REG_CANINTF, 0x00);

    mcp2515_spi_unlock();

    // Reset statistics
    mcp2515_reset_stats();

    // Restore mode (defaults to LISTEN_ONLY if not set)
    can_mode_t target_mode = mcp2515_config.mode;
    if (target_mode == CAN_MODE_CONFIG) {
        target_mode = CAN_MODE_LISTEN_ONLY;  // Don't stay in config
    }
    return mcp2515_set_mode((mcp2515_mode_t)target_mode);
}

// ============================================================================
// Mode control
// ============================================================================

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

    mcp2515_spi_lock();

    // Clear any pending interrupt flags before mode change
    spi_write_register_raw(REG_CANINTF, 0x00);

    // Request mode change
    spi_modify_register_raw(REG_CANCTRL, MODE_MASK, mode_bits);

    mcp2515_spi_unlock();

    // Wait for mode change with timeout (up to 50ms)
    // Release mutex during sleep to allow Core1 to drain frames
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        mcp2515_spi_lock();
        uint8_t status = spi_read_register_raw(REG_CANSTAT);
        mcp2515_spi_unlock();
        if ((status & MODE_MASK) == mode_bits) {
            mcp2515_config.mode = (can_mode_t)mode;  // Save for restore after reset
            return true;
        }
    }

    return false;
}

mcp2515_mode_t mcp2515_get_mode(void) {
    mcp2515_spi_lock();
    uint8_t status = spi_read_register_raw(REG_CANSTAT);
    mcp2515_spi_unlock();
    return (mcp2515_mode_t)((status & MODE_MASK) >> 5);
}

bool mcp2515_is_tx_ready(void) {
    // TX only works in NORMAL mode
    return mcp2515_get_mode() == MCP2515_MODE_NORMAL;
}

// ============================================================================
// Timing / Speed
// ============================================================================

bool mcp2515_set_timing(const mcp2515_timing_t *timing) {
    // Get previous mode from config (not register - avoids extra SPI read)
    uint8_t prev_mode_bits;
    switch (mcp2515_config.mode) {
        case CAN_MODE_NORMAL:      prev_mode_bits = MODE_NORMAL; break;
        case CAN_MODE_SLEEP:       prev_mode_bits = MODE_SLEEP; break;
        case CAN_MODE_LOOPBACK:    prev_mode_bits = MODE_LOOPBACK; break;
        case CAN_MODE_LISTEN_ONLY: prev_mode_bits = MODE_LISTEN; break;
        case CAN_MODE_CONFIG:      prev_mode_bits = MODE_CONFIG; break;
        default:                   prev_mode_bits = MODE_LISTEN; break;
    }

    // Try to enter CONFIG mode without reset
    mcp2515_spi_lock();
    spi_modify_register_raw(REG_CANCTRL, MODE_MASK, MODE_CONFIG);
    mcp2515_spi_unlock();

    // Wait for mode change (MCP2515 won't change if TX in progress)
    // Release mutex during sleep to allow Core1 to drain frames
    bool in_config = false;
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        mcp2515_spi_lock();
        uint8_t status = spi_read_register_raw(REG_CANSTAT);
        mcp2515_spi_unlock();
        if ((status & MODE_MASK) == MODE_CONFIG) {
            in_config = true;
            break;
        }
    }

    if (!in_config) {
        return false;
    }

    // Write timing registers and clear flags (atomic block)
    mcp2515_spi_lock();

    spi_write_register_raw(REG_CNF1, timing->cnf1);
    spi_write_register_raw(REG_CNF2, timing->cnf2);
    spi_write_register_raw(REG_CNF3, timing->cnf3);

    // Save timing to config
    mcp2515_config.timing.cnf1 = timing->cnf1;
    mcp2515_config.timing.cnf2 = timing->cnf2;
    mcp2515_config.timing.cnf3 = timing->cnf3;

    // Clear error flags (EFLG) - may have accumulated during wrong-speed operation
    spi_write_register_raw(REG_EFLG, 0x00);

    // Clear interrupt flags
    spi_write_register_raw(REG_CANINTF, 0x00);

    // Note: RXBxCTRL, filters, masks, and CANINTE all preserve their values
    // when entering CONFIG mode - only RESET clears them

    // Request restore to previous mode
    spi_modify_register_raw(REG_CANCTRL, MODE_MASK, prev_mode_bits);

    mcp2515_spi_unlock();

    // Wait for mode change - release mutex during sleep
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        mcp2515_spi_lock();
        uint8_t status = spi_read_register_raw(REG_CANSTAT);
        mcp2515_spi_unlock();
        if ((status & MODE_MASK) == prev_mode_bits) {
            break;
        }
    }

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

    if (mcp2515_set_timing(&timing)) {
        mcp2515_config.speed_bps = speed_bps;
        mcp2515_config.custom_timing = false;
        return true;
    }
    return false;
}

// ============================================================================
// Transmit
// ============================================================================

bool mcp2515_transmit(const can_frame_t *frame) {
    // Acquire SPI mutex for multi-core safety
    mcp2515_spi_lock();

    // TX only works in NORMAL mode - check current mode
    uint8_t canstat = spi_read_register_raw(REG_CANSTAT);
    if ((canstat & MODE_MASK) != MODE_NORMAL) {
        mcp2515_spi_unlock();
        mcp2515_stats.tx_errors++;
        if (mcp2515_tx_callback) mcp2515_tx_callback(false);
        return false;  // Not in NORMAL mode - can't transmit
    }

    // Check if TX buffer 0 is free
    uint8_t status = spi_read_register_raw(REG_TXB0CTRL);
    if (status & 0x08) {
        mcp2515_spi_unlock();
        mcp2515_stats.tx_errors++;
        if (mcp2515_tx_callback) mcp2515_tx_callback(false);
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

    mcp2515_stats.tx_frames++;
    mcp2515_spi_unlock();

    if (mcp2515_tx_callback) mcp2515_tx_callback(true);
    return true;
}

// ============================================================================
// Filters and Masks
// ============================================================================

bool mcp2515_set_filter(uint8_t filter_num, const mcp2515_filter_t *filter) {
    if (filter_num >= CAN_NUM_FILTERS || filter == NULL) {
        return false;
    }

    mcp2515_spi_lock();

    // Must be in config mode to set filters
    uint8_t status = spi_read_register_raw(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        mcp2515_spi_unlock();
        return false;
    }

    uint8_t reg = mcp2515_get_filter_reg(filter_num);
    mcp2515_write_id_registers(reg, filter->id, filter->extended);

    // Store configuration for reapplication after reset
    mcp2515_config.filters[filter_num].id = filter->id;
    mcp2515_config.filters[filter_num].extended = filter->extended;
    mcp2515_config.filters[filter_num].enabled = true;

    mcp2515_spi_unlock();
    return true;
}

bool mcp2515_set_mask(uint8_t mask_num, uint32_t mask, bool extended) {
    if (mask_num >= CAN_NUM_MASKS) {
        return false;
    }

    mcp2515_spi_lock();

    // Must be in config mode to set masks
    uint8_t status = spi_read_register_raw(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        mcp2515_spi_unlock();
        return false;
    }

    uint8_t reg = (mask_num == 0) ? REG_RXM0 : REG_RXM1;
    mcp2515_write_id_registers(reg, mask, extended);

    // Store configuration for reapplication after reset
    mcp2515_config.masks[mask_num].mask = mask;
    mcp2515_config.masks[mask_num].extended = extended;
    mcp2515_config.masks[mask_num].enabled = true;

    mcp2515_spi_unlock();
    return true;
}

void mcp2515_enable_filters(void) {
    mcp2515_spi_lock();

    // Configure RXB0 and RXB1 to use filters (clear RXM bits)
    // RXM[1:0] = 00 means use filters and masks
    // Preserve rollover setting
    uint8_t rxb0 = mcp2515_config.rollover_enabled ? 0x04 : 0x00;
    spi_write_register_raw(REG_RXB0CTRL, rxb0);  // Filter mode +/- rollover
    spi_write_register_raw(REG_RXB1CTRL, 0x00);  // Filter mode

    // Mark filters as enabled for reapplication after reset
    mcp2515_config.filters_active = true;

    mcp2515_spi_unlock();
}

void mcp2515_clear_filters(void) {
    // Use mode from config (not register read)
    can_mode_t prev_mode = mcp2515_config.mode;

    mcp2515_set_mode(MCP2515_MODE_CONFIG);

    // Set accept-all mode, preserve rollover setting
    uint8_t rxb0 = mcp2515_config.rollover_enabled ? 0x64 : 0x60;
    spi_write_register_raw(REG_RXB0CTRL, rxb0);  // Accept all +/- rollover
    spi_write_register_raw(REG_RXB1CTRL, 0x60);  // Accept all

    // Restore previous mode
    mcp2515_set_mode((mcp2515_mode_t)prev_mode);

    // Clear stored filter configuration
    for (int i = 0; i < CAN_NUM_FILTERS; i++) {
        mcp2515_config.filters[i].enabled = false;
    }
    for (int i = 0; i < CAN_NUM_MASKS; i++) {
        mcp2515_config.masks[i].enabled = false;
    }
    mcp2515_config.filters_active = false;
}

// ============================================================================
// Error handling
// ============================================================================

uint8_t mcp2515_get_error_flags(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_EFLG);
    mcp2515_spi_unlock();
    return val;
}

mcp2515_error_state_t mcp2515_get_error_state(void) {
    mcp2515_spi_lock();
    uint8_t eflg = spi_read_register_raw(REG_EFLG);
    mcp2515_spi_unlock();

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

bool mcp2515_check_and_recover_errors(void) {
    mcp2515_spi_lock();

    // Check if error interrupt flag is set
    uint8_t canintf = spi_read_register_raw(REG_CANINTF);
    if (!(canintf & 0x20)) {  // ERRIF bit
        mcp2515_spi_unlock();
        return false;  // No error
    }

    // Read error flags to determine error type
    uint8_t eflg = spi_read_register_raw(REG_EFLG);

    // Clear the error interrupt flag
    spi_modify_register_raw(REG_CANINTF, 0x20, 0x00);

    mcp2515_spi_unlock();

    // Determine error state
    mcp2515_error_state_t state;
    if (eflg & 0x20) {  // TXBO - Bus-off
        state = CAN_STATE_BUS_OFF;
    } else if (eflg & 0x18) {  // TXEP or RXEP
        state = CAN_STATE_ERROR_PASSIVE;
    } else if (eflg & 0x07) {  // TXWAR, RXWAR, or EWARN
        state = CAN_STATE_ERROR_WARNING;
    } else {
        state = CAN_STATE_ERROR_ACTIVE;
    }

    // Track bus errors in stats
    mcp2515_stats.bus_errors++;

    // Auto-recover from bus-off
    bool recovered = false;
    if (state == CAN_STATE_BUS_OFF) {
        // Automatic bus-off recovery: reset and restore configuration
        recovered = mcp2515_reset_and_restore();
    }

    // Notify via callback
    if (mcp2515_error_callback) {
        mcp2515_error_callback(state, recovered);
    }

    return true;  // Error was detected
}

// ============================================================================
// Debug registers
// ============================================================================

uint8_t mcp2515_get_canintf(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_CANINTF);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_canstat(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_CANSTAT);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_cnf1(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_CNF1);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_cnf2(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_CNF2);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_cnf3(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_CNF3);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_canctrl(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_CANCTRL);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_txb0ctrl(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_TXB0CTRL);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_rxb0ctrl(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_RXB0CTRL);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_rxb1ctrl(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_RXB1CTRL);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_tec(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_TEC);
    mcp2515_spi_unlock();
    return val;
}

uint8_t mcp2515_get_rec(void) {
    mcp2515_spi_lock();
    uint8_t val = spi_read_register_raw(REG_REC);
    mcp2515_spi_unlock();
    return val;
}

// ============================================================================
// One-shot mode
// ============================================================================

void mcp2515_set_oneshot_mode(bool enabled) {
    mcp2515_spi_lock();
    spi_modify_register_raw(REG_CANCTRL, CANCTRL_OSM, enabled ? CANCTRL_OSM : 0);
    mcp2515_config.oneshot_enabled = enabled;
    mcp2515_spi_unlock();
}

// ============================================================================
// Statistics
// ============================================================================

void mcp2515_get_stats(mcp2515_stats_t *out) {
    mcp2515_spi_lock();

    // Copy current stats
    out->rx_frames = mcp2515_stats.rx_frames;
    out->tx_frames = mcp2515_stats.tx_frames;
    out->tx_errors = mcp2515_stats.tx_errors;

    // Read overflow/error flags from EFLG register
    uint8_t eflg = spi_read_register_raw(REG_EFLG);
    out->rx_overflows = (eflg & 0xC0) ? 1 : 0;  // RX0OVR or RX1OVR
    out->bus_errors = (eflg & 0x3F) ? 1 : 0;    // Any error flag

    mcp2515_spi_unlock();
}

void mcp2515_reset_stats(void) {
    mcp2515_spi_lock();
    mcp2515_stats.rx_frames = 0;
    mcp2515_stats.tx_frames = 0;
    mcp2515_stats.tx_errors = 0;
    mcp2515_stats.rx_overflows = 0;
    mcp2515_stats.bus_errors = 0;
    mcp2515_spi_unlock();
}

// ============================================================================
// Callbacks
// ============================================================================

void mcp2515_set_tx_callback(mcp2515_tx_cb_t callback) {
    mcp2515_tx_callback = callback;
}

void mcp2515_set_rx_callback(mcp2515_rx_cb_t callback) {
    mcp2515_rx_callback = callback;
}

void mcp2515_set_error_callback(mcp2515_error_cb_t callback) {
    mcp2515_error_callback = callback;
}

// ============================================================================
// Configuration API
// ============================================================================

const can_config_t* mcp2515_get_config(void) {
    return &mcp2515_config;
}

void mcp2515_get_registers(can_registers_t *regs) {
    mcp2515_spi_lock();

    // Timing registers
    regs->cnf1 = spi_read_register_raw(REG_CNF1);
    regs->cnf2 = spi_read_register_raw(REG_CNF2);
    regs->cnf3 = spi_read_register_raw(REG_CNF3);

    // Control/Status
    regs->canstat = spi_read_register_raw(REG_CANSTAT);
    regs->canctrl = spi_read_register_raw(REG_CANCTRL);

    // Error state
    regs->eflg = spi_read_register_raw(REG_EFLG);
    regs->canintf = spi_read_register_raw(REG_CANINTF);
    regs->tec = spi_read_register_raw(REG_TEC);
    regs->rec = spi_read_register_raw(REG_REC);

    // TX buffer control
    regs->txb0ctrl = spi_read_register_raw(REG_TXB0CTRL);
    regs->txb1ctrl = spi_read_register_raw(REG_TXB1CTRL);
    regs->txb2ctrl = spi_read_register_raw(REG_TXB2CTRL);

    // RX buffer control
    regs->rxb0ctrl = spi_read_register_raw(REG_RXB0CTRL);
    regs->rxb1ctrl = spi_read_register_raw(REG_RXB1CTRL);

    mcp2515_spi_unlock();
}

can_config_status_t mcp2515_check_config(void) {
    can_config_status_t status = {0};
    can_registers_t regs;

    mcp2515_get_registers(&regs);

    // Check mode
    uint8_t actual_mode = (regs.canstat >> 5) & 0x07;
    status.mode_mismatch = (actual_mode != (uint8_t)mcp2515_config.mode);

    // Check timing
    status.timing_mismatch = (regs.cnf1 != mcp2515_config.timing.cnf1 ||
                              regs.cnf2 != mcp2515_config.timing.cnf2 ||
                              regs.cnf3 != mcp2515_config.timing.cnf3);

    // Check oneshot mode (OSM is bit 3 of CANCTRL)
    bool actual_oneshot = (regs.canctrl & 0x08) != 0;
    status.oneshot_mismatch = (actual_oneshot != mcp2515_config.oneshot_enabled);

    // Check filter mode (RXM bits 6:5 of RXB0CTRL)
    // RXM=11 (0x60) = accept all, RXM=00 (0x00) = use filters
    bool actual_filters_active = ((regs.rxb0ctrl & 0x60) == 0x00);
    status.filter_mode_mismatch = (actual_filters_active != mcp2515_config.filters_active);

    return status;
}

void mcp2515_set_rollover(bool enabled) {
    mcp2515_spi_lock();

    mcp2515_config.rollover_enabled = enabled;

    // Update RXB0CTRL: BUKT bit is bit 2
    uint8_t rxb0ctrl = spi_read_register_raw(REG_RXB0CTRL);
    if (enabled) {
        rxb0ctrl |= 0x04;   // Set BUKT
    } else {
        rxb0ctrl &= ~0x04;  // Clear BUKT
    }
    spi_write_register_raw(REG_RXB0CTRL, rxb0ctrl);

    mcp2515_spi_unlock();
}

void mcp2515_set_capture_active(bool active) {
    mcp2515_config.capture_active = active;
}

bool mcp2515_get_capture_active(void) {
    return mcp2515_config.capture_active;
}
