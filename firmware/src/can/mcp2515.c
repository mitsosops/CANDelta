#include "mcp2515.h"
#include "candelta_config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"

// Mutex to protect SPI access between cores
static mutex_t spi_mutex;

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

// Mode bits in CANCTRL
#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTEN     0x60
#define MODE_CONFIG     0x80
#define MODE_MASK       0xE0

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

    // Switch to normal mode
    mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

void mcp2515_reset(void) {
    uint8_t cmd = MCP_RESET;

    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_cs_deselect();
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

    // Clear any pending interrupt flags before mode change
    spi_write_register(REG_CANINTF, 0x00);

    // Request mode change
    spi_modify_register(REG_CANCTRL, MODE_MASK, mode_bits);

    // Wait for mode change with timeout (up to 50ms)
    for (int i = 0; i < 50; i++) {
        sleep_ms(1);
        uint8_t status = spi_read_register(REG_CANSTAT);
        if ((status & MODE_MASK) == mode_bits) {
            return true;
        }
    }
    return false;
}

bool mcp2515_set_speed(uint32_t speed_bps) {
    // Reset the MCP2515 to ensure clean state
    mcp2515_reset();
    sleep_ms(5);

    // After reset, chip is in CONFIG mode by default
    // Verify we're in CONFIG mode
    uint8_t status = spi_read_register(REG_CANSTAT);
    if ((status & MODE_MASK) != MODE_CONFIG) {
        return false;
    }

    // CNF values for 16MHz crystal (from arduino-mcp2515 library)
    uint8_t cnf1, cnf2, cnf3;

    switch (speed_bps) {
        case CAN_SPEED_125KBPS:
            cnf1 = 0x03; cnf2 = 0xF0; cnf3 = 0x86;
            break;
        case CAN_SPEED_250KBPS:
            cnf1 = 0x41; cnf2 = 0xF1; cnf3 = 0x85;
            break;
        case CAN_SPEED_500KBPS:
            cnf1 = 0x00; cnf2 = 0xF0; cnf3 = 0x86;
            break;
        case CAN_SPEED_1MBPS:
            cnf1 = 0x00; cnf2 = 0xD0; cnf3 = 0x82;
            break;
        default:
            return false;
    }

    spi_write_register(REG_CNF1, cnf1);
    spi_write_register(REG_CNF2, cnf2);
    spi_write_register(REG_CNF3, cnf3);

    // Reconfigure RX buffers to accept all messages (reset clears these)
    spi_write_register(REG_RXB0CTRL, 0x64);  // Accept all + rollover enabled
    spi_write_register(REG_RXB1CTRL, 0x60);  // Accept all

    // Re-enable RX interrupts
    spi_write_register(REG_CANINTE, 0x03);

    return true;
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

    // Check if TX buffer 0 is free
    uint8_t status = spi_read_register(REG_TXB0CTRL);
    if (status & 0x08) {
        mutex_exit(&spi_mutex);
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

    mutex_exit(&spi_mutex);
    return true;
}

void mcp2515_set_filter(uint8_t filter_num, uint32_t id, uint32_t mask, bool extended) {
    (void)filter_num;
    (void)id;
    (void)mask;
    (void)extended;
    // TODO: Implement filter configuration
}

void mcp2515_clear_filters(void) {
    mcp2515_set_mode(MCP2515_MODE_CONFIG);
    spi_write_register(REG_RXB0CTRL, 0x64);  // Accept all + rollover enabled
    spi_write_register(REG_RXB1CTRL, 0x60);  // Accept all
    mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

uint8_t mcp2515_get_error_flags(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t val = spi_read_register(REG_EFLG);
    mutex_exit(&spi_mutex);
    return val;
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

bool mcp2515_is_ready(void) {
    mutex_enter_blocking(&spi_mutex);
    uint8_t status = spi_read_register(REG_CANSTAT);
    mutex_exit(&spi_mutex);
    return (status & MODE_MASK) == MODE_NORMAL;
}
