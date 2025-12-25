#ifndef CANDELTA_CONFIG_H
#define CANDELTA_CONFIG_H

// CAN Configuration
#define CAN_SPEED_125KBPS   125000
#define CAN_SPEED_250KBPS   250000
#define CAN_SPEED_500KBPS   500000
#define CAN_SPEED_1MBPS     1000000

#define CAN_DEFAULT_SPEED   CAN_SPEED_500KBPS

// MCP2515 SPI Configuration (Adafruit Feather RP2040 CAN)
#define MCP2515_SPI_PORT    spi1
#define MCP2515_PIN_MISO    8
#define MCP2515_PIN_MOSI    15
#define MCP2515_PIN_SCK     14
#define MCP2515_PIN_CS      19
#define MCP2515_PIN_INT     22

#define MCP2515_SPI_SPEED   (10 * 1000 * 1000)  // 10 MHz

// MCP2515 Crystal frequency (Adafruit uses 16MHz)
#define MCP2515_CRYSTAL_FREQ    16000000

// Ring buffer sizes
#define CAN_RX_BUFFER_SIZE  256
#define CMD_BUFFER_SIZE     64

// LED Configuration (Adafruit Feather RP2040 CAN)
#define RED_LED_PIN         13      // On-board red LED
#define NEOPIXEL_PIN        21      // NeoPixel data line
#define NEOPIXEL_POWER_PIN  20      // NeoPixel power enable

// Protocol version
#define PROTOCOL_VERSION    0x01

#endif // CANDELTA_CONFIG_H
