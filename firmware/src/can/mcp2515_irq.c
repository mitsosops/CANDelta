// MCP2515 Interrupt-driven RX implementation
// Uses GPIO interrupt on INT pin for lower latency frame reception

#include "mcp2515_internal.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

// ============================================================================
// IRQ-specific state
// ============================================================================

// Spinlock for SPI access from IRQ context (mutex can't be used in IRQ)
static spin_lock_t *spi_spinlock;
static uint32_t spi_spinlock_num;

// Ring buffer for frames received in IRQ context
#define IRQ_RX_BUFFER_SIZE 128  // Must be power of 2
static can_frame_t irq_rx_buffer[IRQ_RX_BUFFER_SIZE];
static volatile uint32_t irq_rx_head = 0;  // Written by IRQ
static volatile uint32_t irq_rx_tail = 0;  // Read by main code

// Flag to track if IRQ mode is initialized
static bool irq_mode_initialized = false;

// ============================================================================
// Helper to read one frame from MCP2515 (called from IRQ context)
// ============================================================================

static inline void read_rx_buffer(uint8_t cmd, volatile uint32_t *head) {
    uint32_t next_head = (*head + 1) & (IRQ_RX_BUFFER_SIZE - 1);
    
    uint8_t buf[13];
    spi_cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
    spi_cs_deselect();
    
    if (next_head != irq_rx_tail) {  // Buffer not full
        mcp2515_parse_rx_buffer(buf, &irq_rx_buffer[*head]);
        irq_rx_buffer[*head].timestamp_us = time_us_64();
        __dmb();
        *head = next_head;
        mcp2515_stats.rx_frames++;
    } else {
        mcp2515_stats.rx_overflows++;
    }
}

// ============================================================================
// GPIO Interrupt Handler
// ============================================================================

static void mcp2515_gpio_irq_handler(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;

    // Acquire spinlock (safe in IRQ context)
    uint32_t save = spin_lock_blocking(spi_spinlock);

    // Loop while INT is low - handles frames arriving during ISR
    while (!gpio_get(MCP2515_PIN_INT)) {
        // Use fast READ_STATUS (2 bytes) instead of reading CANINTF (3 bytes)
        uint8_t status = spi_read_status_raw();

        // Check RX0IF (bit 0) and RX1IF (bit 1)
        if (status & 0x01) {
            read_rx_buffer(MCP_READ_RX0, &irq_rx_head);
        }
        if (status & 0x02) {
            read_rx_buffer(MCP_READ_RX1, &irq_rx_head);
        }
        
        // If no RX flags set, break to avoid infinite loop
        if ((status & 0x03) == 0) {
            break;
        }
    }

    spin_unlock(spi_spinlock, save);
}

// ============================================================================
// IRQ Mode Initialization
// ============================================================================

void mcp2515_irq_init(void) {
    if (irq_mode_initialized) return;

    // Claim and initialize spinlock
    spi_spinlock_num = spin_lock_claim_unused(true);
    spi_spinlock = spin_lock_instance(spi_spinlock_num);

    // Reset buffer pointers
    irq_rx_head = 0;
    irq_rx_tail = 0;

    // Make sure INT pin is configured as input
    gpio_init(MCP2515_PIN_INT);
    gpio_set_dir(MCP2515_PIN_INT, GPIO_IN);
    gpio_pull_up(MCP2515_PIN_INT);  // INT is active low, pull high

    // Clear any pending RX data in MCP2515 by reading both buffers
    mcp2515_spi_lock();
    uint8_t status = spi_read_status_raw();
    if (status & 0x01) {
        uint8_t cmd = MCP_READ_RX0;
        uint8_t buf[13];
        spi_cs_select();
        spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
        spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
        spi_cs_deselect();
    }
    if (status & 0x02) {
        uint8_t cmd = MCP_READ_RX1;
        uint8_t buf[13];
        spi_cs_select();
        spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
        spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
        spi_cs_deselect();
    }
    mcp2515_spi_unlock();

    // Configure GPIO interrupt on INT pin (falling edge)
    gpio_set_irq_enabled_with_callback(MCP2515_PIN_INT, GPIO_IRQ_LEVEL_LOW,
                                        true, mcp2515_gpio_irq_handler);

    irq_mode_initialized = true;
}

void mcp2515_irq_deinit(void) {
    if (!irq_mode_initialized) return;

    // Disable GPIO interrupt
    gpio_set_irq_enabled(MCP2515_PIN_INT, GPIO_IRQ_LEVEL_LOW, false);

    // Release spinlock
    spin_lock_unclaim(spi_spinlock_num);

    irq_mode_initialized = false;
}

// ============================================================================
// IRQ-based receive functions
// ============================================================================

bool mcp2515_receive(can_frame_t *frame) {
    if (irq_rx_head == irq_rx_tail) {
        return false;
    }

    *frame = irq_rx_buffer[irq_rx_tail];
    __dmb();
    irq_rx_tail = (irq_rx_tail + 1) & (IRQ_RX_BUFFER_SIZE - 1);

    if (mcp2515_rx_callback) {
        mcp2515_rx_callback(frame);
    }

    return true;
}

int mcp2515_receive_all(can_frame_t *frames, int max_frames) {
    int count = 0;

    while (count < max_frames && irq_rx_head != irq_rx_tail) {
        frames[count] = irq_rx_buffer[irq_rx_tail];
        __dmb();
        irq_rx_tail = (irq_rx_tail + 1) & (IRQ_RX_BUFFER_SIZE - 1);

        if (mcp2515_rx_callback) {
            mcp2515_rx_callback(&frames[count]);
        }
        count++;
    }

    return count;
}

// ============================================================================
// IRQ buffer status
// ============================================================================

uint32_t mcp2515_irq_buffer_count(void) {
    return (irq_rx_head - irq_rx_tail) & (IRQ_RX_BUFFER_SIZE - 1);
}

bool mcp2515_irq_buffer_empty(void) {
    return irq_rx_head == irq_rx_tail;
}

bool mcp2515_irq_buffer_full(void) {
    return ((irq_rx_head + 1) & (IRQ_RX_BUFFER_SIZE - 1)) == irq_rx_tail;
}
