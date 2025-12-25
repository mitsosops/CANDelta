#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "candelta_config.h"
#include "can/mcp2515.h"
#include "usb/usb_comm.h"
#include "protocol/commands.h"
#include "led/led.h"

// Shared state between cores
volatile bool g_capture_active = false;

// Core 1: CAN frame capture (time-critical)
void core1_entry(void) {
    while (true) {
        if (g_capture_active) {
            can_frame_t frame;
            if (mcp2515_receive(&frame)) {
                // Add timestamp and queue for transmission
                frame.timestamp_us = time_us_64();
                usb_queue_frame(&frame);
            }
        }
        // Small delay to prevent busy-waiting when not capturing
        if (!g_capture_active) {
            sleep_us(100);
        }
    }
}

// Core 0: USB communication and command processing
int main(void) {
    // Initialize stdio (USB CDC)
    stdio_init_all();

    // Initialize subsystems
    mcp2515_init();
    usb_comm_init();
    commands_init();
    led_init();

    // Launch Core 1 for CAN capture
    multicore_launch_core1(core1_entry);

    // Main loop: process USB commands and send queued frames
    while (true) {
        // Process incoming commands
        commands_process();

        // Send queued CAN frames over USB
        usb_transmit_queued();

        // Update LED state (handles flash timeout)
        led_update();

        // Yield to USB stack
        sleep_us(100);
    }

    return 0;
}
