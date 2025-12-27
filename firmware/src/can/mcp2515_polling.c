// MCP2515 Polling-based RX implementation
// Optimized version using READ_STATUS command for faster status checks

#include "mcp2515_internal.h"

// ============================================================================
// Polling-based receive functions
// ============================================================================

bool mcp2515_receive(can_frame_t *frame) {
    // Delegate to mcp2515_receive_all() for consistent behavior
    // (timestamp, stats, callback all handled in receive_all)
    return mcp2515_receive_all(frame, 1) > 0;
}

int mcp2515_receive_all(can_frame_t *frames, int max_frames) {
    int count = 0;

    // Fast check: if INT pin is high, no frames pending
    if (gpio_get(MCP2515_PIN_INT)) {
        return 0;
    }

    // Acquire SPI mutex once for the entire batch read
    mcp2515_spi_lock();

    // Loop while we have buffer space - check status each iteration
    while (count < max_frames) {
        // Use fast READ_STATUS (2 bytes) instead of reading CANINTF (3 bytes)
        uint8_t status = spi_read_status_raw();
        uint8_t rx_flags = status & 0x03;  // RX0IF and RX1IF

        if (rx_flags == 0) {
            // No more frames pending
            break;
        }

        // Read RXB0 if has data
        if (rx_flags & 0x01) {
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
        if (rx_flags & 0x02) {
            uint8_t buf[13];
            mcp2515_read_rxb(MCP_READ_RX1, buf);

            mcp2515_parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            mcp2515_stats.rx_frames++;
            if (mcp2515_rx_callback) mcp2515_rx_callback(&frames[count]);
            count++;
        }
    }

    mcp2515_spi_unlock();
    return count;
}
