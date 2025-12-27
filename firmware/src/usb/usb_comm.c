#include "usb_comm.h"
#include "candelta_config.h"
#include "led/led.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "hardware/sync.h"  // For __dmb() and spin_lock
#include <stdio.h>
#include <string.h>

// ============================================================================
// Lock-free SPSC Ring Buffer
// Core 1 (producer) writes frames, Core 0 (consumer) reads and sends to USB
// ============================================================================

// Buffer size must be power-of-2 for efficient bitmask operations
#define BUFFER_MASK (CAN_RX_BUFFER_SIZE - 1)
_Static_assert((CAN_RX_BUFFER_SIZE & BUFFER_MASK) == 0, "CAN_RX_BUFFER_SIZE must be power of 2");

static can_frame_t frame_buffer[CAN_RX_BUFFER_SIZE];
static volatile uint32_t buffer_head = 0;  // Written by Core 1 (producer)
static volatile uint32_t buffer_tail = 0;  // Written by Core 0 (consumer)

// Spinlock for atomic stats reset (not used in hot path)
static spin_lock_t *stats_spinlock;
static uint32_t stats_spinlock_num;

// Frame counters - each counter is only written by one core to avoid contention
static volatile uint32_t rx_frame_count = 0;       // Core 1 writes
static volatile uint32_t tx_to_host_count = 0;     // Core 0 writes
static volatile uint32_t dropped_frame_count = 0;  // Core 1 writes

// Performance tracking
static volatile uint32_t fps_counter = 0;          // Core 1 writes
static volatile uint32_t current_fps = 0;          // Core 0 writes
static volatile uint32_t peak_fps = 0;             // Core 0 writes
static absolute_time_t last_fps_update;

// ============================================================================
// LED Throttling - flash at most every 100ms instead of every frame
// ============================================================================
static absolute_time_t last_led_flash_time;
#define LED_FLASH_INTERVAL_US (100 * 1000)  // 100ms in microseconds

// ============================================================================
// Flush Batching - batch multiple frames before stdio_flush()
// ============================================================================
static uint32_t frames_since_flush = 0;
static absolute_time_t last_flush_time;
#define FLUSH_FRAME_BATCH 8
#define FLUSH_TIME_US (2 * 1000)  // 2ms max latency

void usb_comm_init(void) {
    // Claim spinlock for stats reset
    stats_spinlock_num = spin_lock_claim_unused(true);
    stats_spinlock = spin_lock_instance(stats_spinlock_num);

    // Initialize timing variables
    last_fps_update = get_absolute_time();
    last_led_flash_time = get_absolute_time();
    last_flush_time = get_absolute_time();
}

// Producer (Core 1) - lock-free
bool usb_queue_frame(const can_frame_t *frame) {
    uint32_t head = buffer_head;
    uint32_t next_head = (head + 1) & BUFFER_MASK;

    // Check if buffer full (read tail without lock - safe for SPSC)
    if (next_head == buffer_tail) {
        dropped_frame_count++;  // Only Core 1 writes this
        return false;
    }

    // Write frame data
    memcpy((void*)&frame_buffer[head], frame, sizeof(can_frame_t));

    // Memory barrier: ensure frame data visible before head update
    __dmb();

    // Publish head (atomic write visible to consumer)
    buffer_head = next_head;

    // Update stats (only Core 1 writes these)
    rx_frame_count++;
    fps_counter++;

    return true;
}

uint32_t usb_get_rx_count(void) {
    return rx_frame_count;
}

// Write frame data with batched flushing (for CAN frame streaming)
static void usb_write_frame(const uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        putchar_raw(data[i]);
    }
    frames_since_flush++;

    absolute_time_t now = get_absolute_time();
    int64_t elapsed = absolute_time_diff_us(last_flush_time, now);

    // Flush on batch size OR time limit
    if (frames_since_flush >= FLUSH_FRAME_BATCH || elapsed >= FLUSH_TIME_US || elapsed < 0) {
        stdio_flush();
        frames_since_flush = 0;
        last_flush_time = now;
    }
}

// Consumer (Core 0) - lock-free
void usb_transmit_queued(void) {
    while (true) {
        uint32_t tail = buffer_tail;

        if (tail == buffer_head) {
            break;  // Empty
        }

        // Memory barrier: AFTER seeing non-empty, BEFORE reading frame data
        // This ensures frame data written by producer is visible to us
        __dmb();

        // Read frame data
        can_frame_t frame = frame_buffer[tail];

        // Memory barrier: ensure frame read completes before updating tail
        __dmb();

        // Publish tail (tells producer this slot is free)
        buffer_tail = (tail + 1) & BUFFER_MASK;

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

        usb_write_frame(packet, idx);
        tx_to_host_count++;  // Only Core 0 writes this

        // LED throttling: flash at most every 100ms
        absolute_time_t now = get_absolute_time();
        int64_t elapsed = absolute_time_diff_us(last_led_flash_time, now);
        if (elapsed >= LED_FLASH_INTERVAL_US || elapsed < 0) {
            led_flash(LED_BLUE, 20);
            last_led_flash_time = now;
        }
    }

    // Flush any remaining data after draining
    if (frames_since_flush > 0) {
        stdio_flush();
        frames_since_flush = 0;
        last_flush_time = get_absolute_time();
    }
}

uint32_t usb_get_tx_to_host_count(void) {
    return tx_to_host_count;
}

uint32_t usb_get_buffer_head(void) {
    return buffer_head;
}

uint32_t usb_get_buffer_tail(void) {
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
    uint32_t head = buffer_head;
    uint32_t tail = buffer_tail;
    uint32_t used = (head - tail) & BUFFER_MASK;
    stats->buffer_utilization = (used * 100) / CAN_RX_BUFFER_SIZE;
}

// Reset all stats atomically using spinlock
// This is the only place that touches both head and tail from the same context
void usb_reset_stats(void) {
    uint32_t save = spin_lock_blocking(stats_spinlock);

    rx_frame_count = 0;
    tx_to_host_count = 0;
    dropped_frame_count = 0;
    fps_counter = 0;
    current_fps = 0;
    peak_fps = 0;
    buffer_head = 0;
    buffer_tail = 0;
    last_fps_update = get_absolute_time();

    spin_unlock(stats_spinlock, save);
}
