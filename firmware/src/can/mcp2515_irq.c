// MCP2515 Interrupt-driven RX implementation
// Signal-only ISR: ISR sets flag, Core 1 drains via SPI
// This avoids blocking SPI in interrupt context for better performance

#include "mcp2515_internal.h"
#include "hardware/sync.h"
#include "hardware/irq.h"

// ============================================================================
// IRQ-specific state
// ============================================================================

// Flag to signal pending data - set by ISR, cleared by drain
static volatile bool irq_pending = false;

// Flag to track if IRQ mode is initialized
static bool irq_mode_initialized = false;

// ============================================================================
// Minimal GPIO Interrupt Handler (Signal-Only)
// Sets flag and disables further interrupts until drained
// ============================================================================

static void mcp2515_gpio_irq_handler(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;

    // Set pending flag
    irq_pending = true;

    // Disable IRQ until drained (prevents re-entry while processing)
    gpio_set_irq_enabled(MCP2515_PIN_INT, GPIO_IRQ_LEVEL_LOW, false);
}

// ============================================================================
// IRQ Mode Initialization
// ============================================================================

void mcp2515_irq_init(void) {
    if (irq_mode_initialized) return;

    // Reset pending flag
    irq_pending = false;

    // Make sure INT pin is configured as input
    gpio_init(MCP2515_PIN_INT);
    gpio_set_dir(MCP2515_PIN_INT, GPIO_IN);
    gpio_pull_up(MCP2515_PIN_INT);  // INT is active low, pull high

    // Clear any pending RX data in MCP2515 by reading both buffers
    mcp2515_spi_lock();
    uint8_t status = spi_read_status_raw();
    if (status & 0x01) {
        uint8_t buf[13];
        mcp2515_read_rxb(MCP_READ_RX0, buf);
    }
    if (status & 0x02) {
        uint8_t buf[13];
        mcp2515_read_rxb(MCP_READ_RX1, buf);
    }
    mcp2515_spi_unlock();

    // Configure GPIO interrupt on INT pin (level low)
    gpio_set_irq_enabled_with_callback(MCP2515_PIN_INT, GPIO_IRQ_LEVEL_LOW,
                                        true, mcp2515_gpio_irq_handler);

    irq_mode_initialized = true;
}

void mcp2515_irq_deinit(void) {
    if (!irq_mode_initialized) return;

    // Disable GPIO interrupt
    gpio_set_irq_enabled(MCP2515_PIN_INT, GPIO_IRQ_LEVEL_LOW, false);

    irq_mode_initialized = false;
}

// ============================================================================
// IRQ-based drain function (called from Core 1 loop)
// This does the actual SPI work that was previously in the ISR
// ============================================================================

int mcp2515_irq_drain(can_frame_t *frames, int max_frames) {
    // Quick check: if no pending flag, nothing to do
    if (!irq_pending) {
        return 0;
    }

    int count = 0;

    // Acquire the SAME mutex used by all other SPI operations
    // Note: IRQ is already disabled by the ISR, so we just need the mutex
    mutex_enter_blocking(&mcp2515_spi_mutex);

    // Drain while INT is low and we have buffer space
    while (count < max_frames && !gpio_get(MCP2515_PIN_INT)) {
        uint8_t status = spi_read_status_raw();

        // Read RXB0 if has data
        if (status & 0x01) {
            uint8_t buf[13];
            mcp2515_read_rxb(MCP_READ_RX0, buf);
            mcp2515_parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            mcp2515_stats.rx_frames++;
            if (mcp2515_rx_callback) mcp2515_rx_callback(&frames[count]);
            count++;

            if (count >= max_frames) break;
        }

        // Read RXB1 if has data
        if (count < max_frames && (status & 0x02)) {
            uint8_t buf[13];
            mcp2515_read_rxb(MCP_READ_RX1, buf);
            mcp2515_parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            mcp2515_stats.rx_frames++;
            if (mcp2515_rx_callback) mcp2515_rx_callback(&frames[count]);
            count++;
        }

        // If no RX flags set, break to avoid infinite loop
        if ((status & 0x03) == 0) {
            break;
        }
    }

    mutex_exit(&mcp2515_spi_mutex);

    // Clear pending flag and re-enable IRQ
    irq_pending = false;
    gpio_set_irq_enabled(MCP2515_PIN_INT, GPIO_IRQ_LEVEL_LOW, true);

    return count;
}

// ============================================================================
// Receive functions for API compatibility
// These delegate to mcp2515_irq_drain() for consistent behavior
// ============================================================================

bool mcp2515_receive(can_frame_t *frame) {
    return mcp2515_irq_drain(frame, 1) > 0;
}

int mcp2515_receive_all(can_frame_t *frames, int max_frames) {
    return mcp2515_irq_drain(frames, max_frames);
}

// ============================================================================
// IRQ status functions
// ============================================================================

bool mcp2515_irq_is_pending(void) {
    return irq_pending;
}
