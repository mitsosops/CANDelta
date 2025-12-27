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
    // Check both irq_pending flag AND INT pin state
    // INT pin is the source of truth - if it's low, frames are waiting
    // irq_pending might be false if GPIO IRQ was disabled when INT went low
    if (!irq_pending && gpio_get(MCP2515_PIN_INT)) {
        return 0;  // Nothing pending AND INT is high - truly nothing to do
    }

    // Acquire mutex and drain using shared helper
    mcp2515_spi_lock();
    int count = mcp2515_drain_rx_locked(frames, max_frames);
    mcp2515_spi_unlock();

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
