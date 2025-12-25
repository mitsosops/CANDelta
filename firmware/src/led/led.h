#ifndef LED_H
#define LED_H

#include <stdint.h>

// NeoPixel colors (GRB format) - ~10% brightness to avoid blinding
#define LED_OFF     0x000000
#define LED_GREEN   0x1A0000  // GRB: G=1A, R=00, B=00
#define LED_BLUE    0x00001A  // GRB: G=00, R=00, B=1A
#define LED_RED     0x001A00  // GRB: G=00, R=1A, B=00

// Initialize LEDs (red power LED on, NeoPixel off)
void led_init(void);

// Flash NeoPixel with color for duration_ms (non-blocking)
void led_flash(uint32_t color, uint16_t duration_ms);

// Update LED state (call from main loop)
void led_update(void);

#endif // LED_H
