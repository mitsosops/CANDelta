#include "commands.h"
#include "candelta_config.h"
#include "can/mcp2515.h"
#include "usb/usb_comm.h"
#include "led/led.h"
#include <string.h>

// External state from main.c
extern volatile bool g_capture_active;

// Internal state
static uint32_t g_can_speed = CAN_DEFAULT_SPEED;
static uint32_t g_rx_count = 0;
static uint32_t g_tx_count = 0;

// Command buffer
static uint8_t cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_len = 0;

static void send_response(response_code_t code, const uint8_t *data, uint8_t len) {
    uint8_t packet[64];
    int idx = 0;

    packet[idx++] = 0x02;  // STX
    packet[idx++] = code;
    packet[idx++] = len;

    for (int i = 0; i < len && idx < 62; i++) {
        packet[idx++] = data[i];
    }

    packet[idx++] = 0x03;  // ETX

    usb_write(packet, idx);
}

static void send_ack(void) {
    send_response(RSP_ACK, NULL, 0);
}

static void send_nak(uint8_t error_code) {
    send_response(RSP_NAK, &error_code, 1);
}

static void handle_command(uint8_t opcode, const uint8_t *params, uint8_t param_len) {
    // Flash green on RX command
    led_flash(LED_GREEN, 50);

    switch (opcode) {
        case CMD_PING:
            send_ack();
            break;

        case CMD_GET_VERSION: {
            static const uint8_t version[] = {PROTOCOL_VERSION, 0, 1, 0};  // v0.1.0
            send_response(RSP_VERSION, version, 4);
            break;
        }

        case CMD_GET_STATUS: {
            device_status_t status;
            commands_get_status(&status);

            // Manual serialization to avoid struct padding issues
            uint8_t buf[16];
            int idx = 0;
            buf[idx++] = status.protocol_version;
            buf[idx++] = status.mode;
            buf[idx++] = (status.can_speed >> 0) & 0xFF;
            buf[idx++] = (status.can_speed >> 8) & 0xFF;
            buf[idx++] = (status.can_speed >> 16) & 0xFF;
            buf[idx++] = (status.can_speed >> 24) & 0xFF;
            buf[idx++] = status.capture_active ? 1 : 0;
            buf[idx++] = status.error_flags;
            buf[idx++] = (status.rx_frame_count >> 0) & 0xFF;
            buf[idx++] = (status.rx_frame_count >> 8) & 0xFF;
            buf[idx++] = (status.rx_frame_count >> 16) & 0xFF;
            buf[idx++] = (status.rx_frame_count >> 24) & 0xFF;
            buf[idx++] = (status.tx_frame_count >> 0) & 0xFF;
            buf[idx++] = (status.tx_frame_count >> 8) & 0xFF;
            buf[idx++] = (status.tx_frame_count >> 16) & 0xFF;
            buf[idx++] = (status.tx_frame_count >> 24) & 0xFF;

            send_response(RSP_STATUS, buf, 16);
            break;
        }

        case CMD_START_CAPTURE:
            g_capture_active = true;
            send_ack();
            break;

        case CMD_STOP_CAPTURE:
            g_capture_active = false;
            send_ack();
            break;

        case CMD_SET_SPEED:
            if (param_len >= 4) {
                uint32_t speed = params[0] | (params[1] << 8) |
                                (params[2] << 16) | (params[3] << 24);

                bool was_capturing = g_capture_active;
                g_capture_active = false;

                if (mcp2515_set_speed(speed)) {
                    g_can_speed = speed;
                    mcp2515_set_mode(MCP2515_MODE_NORMAL);
                    send_ack();
                } else {
                    send_nak(0x01);  // Invalid speed
                }

                g_capture_active = was_capturing;
            } else {
                send_nak(0x02);  // Invalid params
            }
            break;

        case CMD_SET_FILTER:
            // TODO: Implement filter setting
            send_ack();
            break;

        case CMD_CLEAR_FILTERS:
            mcp2515_clear_filters();
            send_ack();
            break;

        case CMD_SET_MODE:
            if (param_len >= 1) {
                if (mcp2515_set_mode((mcp2515_mode_t)params[0])) {
                    send_ack();
                } else {
                    send_nak(0x03);  // Mode change failed
                }
            } else {
                send_nak(0x02);
            }
            break;

        case CMD_TRANSMIT_FRAME:
            if (param_len >= 6) {
                can_frame_t frame;
                frame.id = params[0] | (params[1] << 8) |
                          (params[2] << 16) | (params[3] << 24);
                frame.extended = (params[4] & 0x01) != 0;
                frame.rtr = (params[4] & 0x02) != 0;
                frame.dlc = params[5];
                if (frame.dlc > 8) frame.dlc = 8;

                for (int i = 0; i < frame.dlc && (6 + i) < param_len; i++) {
                    frame.data[i] = params[6 + i];
                }

                if (mcp2515_transmit(&frame)) {
                    g_tx_count++;
                    led_flash(LED_BLUE, 50);  // Flash blue on TX
                    send_ack();
                } else {
                    send_nak(0x04);  // TX failed
                }
            } else {
                send_nak(0x02);
            }
            break;

        default:
            send_nak(0xFF);  // Unknown command
            break;
    }
}

void commands_init(void) {
    cmd_len = 0;
}

void commands_process(void) {
    uint8_t rx_buf[64];
    int rx_len = usb_read(rx_buf, sizeof(rx_buf));

    for (int i = 0; i < rx_len; i++) {
        uint8_t byte = rx_buf[i];

        if (byte == 0x02) {
            // Start of packet
            cmd_len = 0;
        } else if (byte == 0x03) {
            // End of packet - process command
            if (cmd_len >= 2) {
                uint8_t opcode = cmd_buffer[0];
                uint8_t param_len = cmd_buffer[1];

                if (param_len + 2 <= cmd_len) {
                    handle_command(opcode, &cmd_buffer[2], param_len);
                }
            }
            cmd_len = 0;
        } else {
            // Add to buffer
            if (cmd_len < CMD_BUFFER_SIZE) {
                cmd_buffer[cmd_len++] = byte;
            }
        }
    }
}

void commands_get_status(device_status_t *status) {
    status->protocol_version = PROTOCOL_VERSION;
    status->mode = 0;  // TODO: Get actual mode
    status->can_speed = g_can_speed;
    status->capture_active = g_capture_active;
    status->error_flags = mcp2515_get_error_flags();
    status->rx_frame_count = g_rx_count;
    status->tx_frame_count = g_tx_count;
}
