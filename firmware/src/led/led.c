#include "led.h"
#include "candelta_config.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "ws2812.pio.h"

static PIO pio = pio0;
static uint sm;
static absolute_time_t flash_end_time;
static bool flash_active = false;

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

void led_init(void) {
    // Red power LED - always on
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    gpio_put(RED_LED_PIN, 1);

    // NeoPixel power enable
    gpio_init(NEOPIXEL_POWER_PIN);
    gpio_set_dir(NEOPIXEL_POWER_PIN, GPIO_OUT);
    gpio_put(NEOPIXEL_POWER_PIN, 1);

    // Small delay for NeoPixel power to stabilize
    sleep_ms(1);

    // Initialize PIO for WS2812
    sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, NEOPIXEL_PIN, 800000, false);

    // Start with NeoPixel off
    put_pixel(LED_OFF);
}

void led_flash(uint32_t color, uint16_t duration_ms) {
    flash_end_time = make_timeout_time_ms(duration_ms);
    flash_active = true;
    put_pixel(color);
}

void led_update(void) {
    if (flash_active && time_reached(flash_end_time)) {
        put_pixel(LED_OFF);
        flash_active = false;
    }
}
