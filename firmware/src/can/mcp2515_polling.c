// MCP2515 Polling-based RX implementation
// Uses shared drain loop from mcp2515_internal.h

#include "mcp2515_internal.h"

// ============================================================================
// Polling-based receive functions
// ============================================================================

bool mcp2515_receive(can_frame_t *frame) {
    return mcp2515_receive_all(frame, 1) > 0;
}

int mcp2515_receive_all(can_frame_t *frames, int max_frames) {
    // Fast check: if INT pin is high, no frames pending
    if (gpio_get(MCP2515_PIN_INT)) {
        return 0;
    }

    mcp2515_spi_lock();
    int count = mcp2515_drain_rx_locked(frames, max_frames);
    mcp2515_spi_unlock();

    return count;
}
