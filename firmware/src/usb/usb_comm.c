#include "usb_comm.h"
#include "candelta_config.h"
#include "led/led.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include <stdio.h>
#include <string.h>

// Ring buffer for CAN frames (Core 1 writes, Core 0 reads)
static can_frame_t frame_buffer[CAN_RX_BUFFER_SIZE];
static volatile uint16_t buffer_head = 0;
static volatile uint16_t buffer_tail = 0;
static mutex_t buffer_mutex;

// Frame counters
static volatile uint32_t rx_frame_count = 0;
static volatile uint32_t tx_to_host_count = 0;
static volatile uint32_t dropped_frame_count = 0;

// Performance tracking
static volatile uint32_t fps_counter = 0;          // Frames in current second
static volatile uint32_t current_fps = 0;          // Last calculated FPS
static volatile uint32_t peak_fps = 0;             // Maximum FPS observed
static absolute_time_t last_fps_update;

void usb_comm_init(void) {
    mutex_init(&buffer_mutex);
    last_fps_update = get_absolute_time();
}

bool usb_queue_frame(const can_frame_t *frame) {
    mutex_enter_blocking(&buffer_mutex);

    uint16_t next_head = (buffer_head + 1) % CAN_RX_BUFFER_SIZE;

    if (next_head == buffer_tail) {
        // Buffer full - frame dropped
        dropped_frame_count++;
        mutex_exit(&buffer_mutex);
        return false;
    }

    memcpy((void*)&frame_buffer[buffer_head], frame, sizeof(can_frame_t));
    buffer_head = next_head;
    rx_frame_count++;
    fps_counter++;

    mutex_exit(&buffer_mutex);
    return true;
}

uint32_t usb_get_rx_count(void) {
    return rx_frame_count;
}

void usb_transmit_queued(void) {
    while (buffer_tail != buffer_head) {
        mutex_enter_blocking(&buffer_mutex);

        if (buffer_tail == buffer_head) {
            mutex_exit(&buffer_mutex);
            break;
        }

        can_frame_t frame = frame_buffer[buffer_tail];
        buffer_tail = (buffer_tail + 1) % CAN_RX_BUFFER_SIZE;

        mutex_exit(&buffer_mutex);

        // Serialize frame to wire format
        // Format: [STX][opcode][len][timestamp:8][id:4][flags:1][dlc:1][data:0-8][ETX]
        uint8_t packet[32];
        int idx = 0;

        packet[idx++] = 0x02;  // STX
        packet[idx++] = 0x84;  // RSP_CAN_FRAME opcode

        // Calculate payload length: 8 + 4 + 1 + 1 + dlc = 14 + dlc
        uint8_t payload_len = 14 + frame.dlc;
        packet[idx++] = payload_len;

        // Timestamp (8 bytes, little-endian)
        for (int i = 0; i < 8; i++) {
            packet[idx++] = (frame.timestamp_us >> (i * 8)) & 0xFF;
        }

        // CAN ID (4 bytes, little-endian)
        for (int i = 0; i < 4; i++) {
            packet[idx++] = (frame.id >> (i * 8)) & 0xFF;
        }

        // Flags: bit 0 = extended, bit 1 = RTR
        packet[idx++] = (frame.extended ? 0x01 : 0x00) | (frame.rtr ? 0x02 : 0x00);

        // DLC
        packet[idx++] = frame.dlc;

        // Data
        for (int i = 0; i < frame.dlc && i < 8; i++) {
            packet[idx++] = frame.data[i];
        }

        packet[idx++] = 0x03;  // ETX

        usb_write(packet, idx);
        tx_to_host_count++;

        // Flash blue for each CAN frame sent to host
        led_flash(LED_BLUE, 20);
    }
}

uint32_t usb_get_tx_to_host_count(void) {
    return tx_to_host_count;
}

uint16_t usb_get_buffer_head(void) {
    return buffer_head;
}

uint16_t usb_get_buffer_tail(void) {
    return buffer_tail;
}

int usb_read(uint8_t *buffer, int max_len) {
    int count = 0;
    while (count < max_len) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            break;
        }
        buffer[count++] = (uint8_t)c;
    }
    return count;
}

void usb_write(const uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        putchar_raw(data[i]);
    }
    stdio_flush();
}

bool usb_is_connected(void) {
    return stdio_usb_connected();
}

void usb_update_perf_stats(void) {
    // Check if 1 second has passed
    if (absolute_time_diff_us(last_fps_update, get_absolute_time()) >= 1000000) {
        current_fps = fps_counter;
        if (current_fps > peak_fps) {
            peak_fps = current_fps;
        }
        fps_counter = 0;
        last_fps_update = get_absolute_time();
    }
}

void usb_get_perf_stats(perf_stats_t *stats) {
    stats->frames_per_second = current_fps;
    stats->peak_fps = peak_fps;
    stats->dropped_frames = dropped_frame_count;

    // Calculate buffer utilization (0-100%)
    uint16_t head = buffer_head;
    uint16_t tail = buffer_tail;
    uint16_t used;
    if (head >= tail) {
        used = head - tail;
    } else {
        used = CAN_RX_BUFFER_SIZE - tail + head;
    }
    stats->buffer_utilization = (used * 100) / CAN_RX_BUFFER_SIZE;
}

void usb_reset_stats(void) {
    mutex_enter_blocking(&buffer_mutex);
    rx_frame_count = 0;
    tx_to_host_count = 0;
    dropped_frame_count = 0;
    fps_counter = 0;
    current_fps = 0;
    peak_fps = 0;
    buffer_head = 0;
    buffer_tail = 0;
    last_fps_update = get_absolute_time();
    mutex_exit(&buffer_mutex);
}
