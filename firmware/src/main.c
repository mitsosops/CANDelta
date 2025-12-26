#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "candelta_config.h"
#include "can/mcp2515.h"
#include "usb/usb_comm.h"
#include "protocol/commands.h"
#include "led/led.h"

// Shared state between cores
volatile bool g_capture_active = false;

// Error callback - flash red LED on bus errors
static void on_can_error(mcp2515_error_state_t state, bool recovered) {
    if (state == CAN_STATE_BUS_OFF) {
        // Bus-off: long red flash (recovery attempted)
        led_flash(LED_RED, 500);
    } else if (state >= CAN_STATE_ERROR_WARNING) {
        // Warning/Passive: short red flash
        led_flash(LED_RED, 100);
    }
    (void)recovered;  // Could send notification to host in future
}

// Core 1: CAN frame capture (tight polling with optimized receive)
void core1_entry(void) {
    // Larger buffer to handle bursts - mcp2515_receive_all() loops until INT goes high
    can_frame_t frames[8];

    while (true) {
        if (g_capture_active) {
            // Tight polling - check INT pin and read all available frames
            if (!gpio_get(MCP2515_PIN_INT)) {
                int count = mcp2515_receive_all(frames, 8);
                for (int i = 0; i < count; i++) {
                    usb_queue_frame(&frames[i]);
                }
            }
        } else {
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

    // Register error callback for automatic bus-off recovery with LED feedback
    mcp2515_set_error_callback(on_can_error);

    // Launch Core 1 for CAN capture
    multicore_launch_core1(core1_entry);

    // Main loop: process USB commands and send queued frames
    while (true) {
        // Process incoming commands
        commands_process();

        // Send queued CAN frames over USB
        usb_transmit_queued();

        // Check for CAN errors and auto-recover from bus-off
        // This is checked on Core 0 since recovery is heavyweight
        mcp2515_check_and_recover_errors();

        // Update LED state (handles flash timeout)
        led_update();

        // Update performance stats (FPS calculation)
        usb_update_perf_stats();

        // Yield to USB stack
        sleep_us(100);
    }

    return 0;
}
