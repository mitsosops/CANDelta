// MCP2515 Polling-based RX implementation
// Optimized version using READ_STATUS command for faster status checks

#include "mcp2515_internal.h"

// ============================================================================
// Polling-based receive functions
// ============================================================================

bool mcp2515_receive(can_frame_t *frame) {
    // Check interrupt pin (active low) - fast GPIO check before SPI
    if (gpio_get(MCP2515_PIN_INT)) {
        return false;  // No interrupt, no data
    }

    mcp2515_spi_lock();

    // Use fast READ_STATUS command (2 bytes vs 3 for register read)
    uint8_t status = spi_read_status_raw();

    if (status & 0x01) {  // RX0IF bit
        // RX buffer 0 has data - READ_RX0 command auto-clears RX0IF
        uint8_t cmd = MCP_READ_RX0;
        uint8_t buf[13];

        spi_cs_select();
        spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
        spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
        spi_cs_deselect();

        mcp2515_parse_rx_buffer(buf, frame);

        mcp2515_spi_unlock();
        return true;
    }

    if (status & 0x02) {  // RX1IF bit
        // RX buffer 1 has data - READ_RX1 command auto-clears RX1IF
        uint8_t cmd = MCP_READ_RX1;
        uint8_t buf[13];

        spi_cs_select();
        spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
        spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
        spi_cs_deselect();

        mcp2515_parse_rx_buffer(buf, frame);

        mcp2515_spi_unlock();
        return true;
    }

    mcp2515_spi_unlock();
    return false;
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
            uint8_t cmd = MCP_READ_RX0;
            uint8_t buf[13];

            spi_cs_select();
            spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
            spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
            spi_cs_deselect();

            mcp2515_parse_rx_buffer(buf, &frames[count]);
            frames[count].timestamp_us = time_us_64();
            mcp2515_stats.rx_frames++;
            if (mcp2515_rx_callback) mcp2515_rx_callback(&frames[count]);
            count++;

            if (count >= max_frames) break;
        }

        // Read RXB1 if has data
        if (rx_flags & 0x02) {
            uint8_t cmd = MCP_READ_RX1;
            uint8_t buf[13];

            spi_cs_select();
            spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
            spi_read_blocking(MCP2515_SPI_PORT, 0, buf, 13);
            spi_cs_deselect();

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
