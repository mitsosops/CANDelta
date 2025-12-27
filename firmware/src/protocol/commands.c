#include "commands.h"
#include "candelta_config.h"
#include "can/mcp2515.h"
#include "usb/usb_comm.h"
#include "led/led.h"
#include "pico/unique_id.h"
#include <string.h>

// External state from main.c
extern volatile bool g_capture_active;

// Internal state
static uint32_t g_tx_count = 0;

// Calculate speed from CNF registers (returns 0 if unknown/custom timing)
static uint32_t calculate_speed_from_cnf(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3) {
    // Known CNF values for 16MHz crystal
    if (cnf1 == 0x03 && cnf2 == 0xF0 && cnf3 == 0x86) return 125000;
    if (cnf1 == 0x41 && cnf2 == 0xF1 && cnf3 == 0x85) return 250000;
    if (cnf1 == 0x00 && cnf2 == 0xF0 && cnf3 == 0x86) return 500000;
    if (cnf1 == 0x00 && cnf2 == 0xD0 && cnf3 == 0x82) return 1000000;
    return 0;  // Custom timing - can't determine speed
}

// Command buffer and parser state
static uint8_t cmd_buffer[CMD_BUFFER_SIZE];
static uint8_t cmd_len = 0;
static uint8_t cmd_expected_len = 0;  // Expected payload length
static enum { WAIT_STX, READ_OPCODE, READ_LEN, READ_PAYLOAD, WAIT_ETX, DISCARD_UNTIL_ETX } parse_state = WAIT_STX;

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
    // Flash green on USB command RX
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

        case CMD_DEBUG: {
            // Return buffer debug info + MCP2515 registers
            uint8_t buf[12];
            uint16_t head = usb_get_buffer_head();
            uint16_t tail = usb_get_buffer_tail();
            buf[0] = head & 0xFF;
            buf[1] = (head >> 8) & 0xFF;
            buf[2] = tail & 0xFF;
            buf[3] = (tail >> 8) & 0xFF;
            buf[4] = usb_get_rx_count() & 0xFF;
            buf[5] = usb_get_tx_to_host_count() & 0xFF;
            buf[6] = g_capture_active ? 1 : 0;
            buf[7] = mcp2515_get_canintf();   // Interrupt flags
            buf[8] = mcp2515_get_canstat();   // Status/mode
            buf[9] = mcp2515_get_error_flags();
            buf[10] = mcp2515_get_cnf1();  // Should be 0x00 for 500kbps
            buf[11] = mcp2515_get_txb0ctrl(); // TX buffer 0 ctrl (TXREQ, TXERR, etc.)
            send_response(RSP_DEBUG, buf, 12);
            break;
        }

        case CMD_GET_PERF_STATS: {
            perf_stats_t stats;
            usb_get_perf_stats(&stats);

            // Serialize: fps(4) + peak_fps(4) + dropped(4) + util(1) = 13 bytes
            uint8_t buf[13];
            buf[0] = (stats.frames_per_second >> 0) & 0xFF;
            buf[1] = (stats.frames_per_second >> 8) & 0xFF;
            buf[2] = (stats.frames_per_second >> 16) & 0xFF;
            buf[3] = (stats.frames_per_second >> 24) & 0xFF;
            buf[4] = (stats.peak_fps >> 0) & 0xFF;
            buf[5] = (stats.peak_fps >> 8) & 0xFF;
            buf[6] = (stats.peak_fps >> 16) & 0xFF;
            buf[7] = (stats.peak_fps >> 24) & 0xFF;
            buf[8] = (stats.dropped_frames >> 0) & 0xFF;
            buf[9] = (stats.dropped_frames >> 8) & 0xFF;
            buf[10] = (stats.dropped_frames >> 16) & 0xFF;
            buf[11] = (stats.dropped_frames >> 24) & 0xFF;
            buf[12] = stats.buffer_utilization;

            send_response(RSP_PERF_STATS, buf, 13);
            break;
        }

        case CMD_GET_DEVICE_ID: {
            pico_unique_board_id_t id;
            pico_get_unique_board_id(&id);
            send_response(RSP_DEVICE_ID, id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
            break;
        }

        case CMD_GET_ERROR_COUNTERS: {
            uint8_t buf[3];
            buf[0] = mcp2515_get_tec();
            buf[1] = mcp2515_get_rec();
            buf[2] = (uint8_t)mcp2515_get_error_state();
            send_response(RSP_ERROR_COUNTERS, buf, 3);
            break;
        }

        case CMD_GET_CONFIG: {
            // Return full configuration
            // Format: speed(4) + timing(3) + mode(1) + flags(1) + filters(6*5) + masks(2*5) = 49 bytes
            const can_config_t *cfg = mcp2515_get_config();
            uint8_t buf[49];
            int idx = 0;

            // Speed (4 bytes, LE)
            buf[idx++] = (cfg->speed_bps >> 0) & 0xFF;
            buf[idx++] = (cfg->speed_bps >> 8) & 0xFF;
            buf[idx++] = (cfg->speed_bps >> 16) & 0xFF;
            buf[idx++] = (cfg->speed_bps >> 24) & 0xFF;

            // Timing (3 bytes)
            buf[idx++] = cfg->timing.cnf1;
            buf[idx++] = cfg->timing.cnf2;
            buf[idx++] = cfg->timing.cnf3;

            // Mode (1 byte)
            buf[idx++] = (uint8_t)cfg->mode;

            // Flags (1 byte): bit0=custom_timing, bit1=filters_active, bit2=rollover, bit3=oneshot, bit4=capture
            buf[idx++] = (cfg->custom_timing ? 0x01 : 0) |
                         (cfg->filters_active ? 0x02 : 0) |
                         (cfg->rollover_enabled ? 0x04 : 0) |
                         (cfg->oneshot_enabled ? 0x08 : 0) |
                         (cfg->capture_active ? 0x10 : 0);

            // Filters (6 * 5 bytes each): id(4) + flags(1)
            for (int i = 0; i < CAN_NUM_FILTERS; i++) {
                buf[idx++] = (cfg->filters[i].id >> 0) & 0xFF;
                buf[idx++] = (cfg->filters[i].id >> 8) & 0xFF;
                buf[idx++] = (cfg->filters[i].id >> 16) & 0xFF;
                buf[idx++] = (cfg->filters[i].id >> 24) & 0xFF;
                buf[idx++] = (cfg->filters[i].enabled ? 0x01 : 0) |
                             (cfg->filters[i].extended ? 0x02 : 0);
            }

            // Masks (2 * 5 bytes each): mask(4) + flags(1)
            for (int i = 0; i < CAN_NUM_MASKS; i++) {
                buf[idx++] = (cfg->masks[i].mask >> 0) & 0xFF;
                buf[idx++] = (cfg->masks[i].mask >> 8) & 0xFF;
                buf[idx++] = (cfg->masks[i].mask >> 16) & 0xFF;
                buf[idx++] = (cfg->masks[i].mask >> 24) & 0xFF;
                buf[idx++] = (cfg->masks[i].enabled ? 0x01 : 0) |
                             (cfg->masks[i].extended ? 0x02 : 0);
            }

            send_response(RSP_CONFIG, buf, idx);
            break;
        }

        case CMD_GET_REGISTERS: {
            // Return raw MCP2515 register values
            // Format: cnf(3) + canstat(1) + canctrl(1) + eflg(1) + canintf(1) + tec(1) + rec(1)
            //         + txbNctrl(3) + rxbNctrl(2) = 15 bytes
            can_registers_t regs;
            mcp2515_get_registers(&regs);

            uint8_t buf[15];
            buf[0] = regs.cnf1;
            buf[1] = regs.cnf2;
            buf[2] = regs.cnf3;
            buf[3] = regs.canstat;
            buf[4] = regs.canctrl;
            buf[5] = regs.eflg;
            buf[6] = regs.canintf;
            buf[7] = regs.tec;
            buf[8] = regs.rec;
            buf[9] = regs.txb0ctrl;
            buf[10] = regs.txb1ctrl;
            buf[11] = regs.txb2ctrl;
            buf[12] = regs.rxb0ctrl;
            buf[13] = regs.rxb1ctrl;
            // Config status (mismatches)
            can_config_status_t cstatus = mcp2515_check_config();
            buf[14] = (cstatus.mode_mismatch ? 0x01 : 0) |
                      (cstatus.timing_mismatch ? 0x02 : 0) |
                      (cstatus.oneshot_mismatch ? 0x04 : 0) |
                      (cstatus.filter_mode_mismatch ? 0x08 : 0);

            send_response(RSP_REGISTERS, buf, 15);
            break;
        }

        case CMD_LIST_COMMANDS: {
            // Return list of (opcode, param_count) pairs
            // param_count = number of logical parameters (not byte count)
            static const uint8_t cmd_list[] = {
                CMD_PING, 0,
                CMD_GET_VERSION, 0,
                CMD_GET_STATUS, 0,
                CMD_DEBUG, 0,
                CMD_GET_PERF_STATS, 0,
                CMD_GET_DEVICE_ID, 0,
                CMD_GET_ERROR_COUNTERS, 0,
                CMD_LIST_COMMANDS, 0,
                CMD_GET_CONFIG, 0,
                CMD_GET_REGISTERS, 0,
                CMD_START_CAPTURE, 0,
                CMD_STOP_CAPTURE, 0,
                CMD_SET_SPEED, 1,        // speed
                CMD_SET_FILTER, 3,       // filter_num, id, extended
                CMD_CLEAR_FILTERS, 0,
                CMD_SET_MODE, 1,         // mode
                CMD_SET_TIMING, 3,       // cnf1, cnf2, cnf3
                CMD_SET_MASK, 3,         // mask_num, mask, extended
                CMD_SET_ONESHOT, 1,      // enabled
                CMD_RESET_CAN, 0,        // reset and restore config
                CMD_TRANSMIT_FRAME, 3,   // id, flags, dlc (+ variable data)
            };
            send_response(RSP_COMMAND_LIST, cmd_list, sizeof(cmd_list));
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
            usb_reset_stats();  // Reset counters for new capture session
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
                    // Speed saved to config in mcp2515_set_speed()
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
            if (param_len >= 6) {
                uint8_t filter_num = params[0];
                mcp2515_filter_t filter;
                filter.id = params[1] | (params[2] << 8) |
                           (params[3] << 16) | (params[4] << 24);
                filter.extended = (params[5] & 0x01) != 0;
                filter.mask = 0xFFFFFFFF;  // Full match by default

                // Save current mode to restore after
                mcp2515_mode_t prev_mode = mcp2515_get_mode();

                // Enter config mode, set filter, enable filtering, restore mode
                if (mcp2515_set_mode(MCP2515_MODE_CONFIG)) {
                    if (mcp2515_set_filter(filter_num, &filter)) {
                        mcp2515_enable_filters();
                        mcp2515_set_mode(prev_mode);
                        send_ack();
                    } else {
                        mcp2515_set_mode(prev_mode);
                        send_nak(0x06);  // Invalid filter
                    }
                } else {
                    send_nak(0x03);  // Mode change failed
                }
            } else {
                send_nak(0x02);
            }
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

        case CMD_SET_TIMING:
            if (param_len >= 3) {
                mcp2515_timing_t timing;
                timing.cnf1 = params[0];
                timing.cnf2 = params[1];
                timing.cnf3 = params[2];

                bool was_capturing = g_capture_active;
                g_capture_active = false;

                if (mcp2515_set_timing(&timing)) {
                    // mcp2515_set_timing() preserves the current mode
                    send_ack();
                } else {
                    send_nak(0x05);  // Timing config failed
                }

                g_capture_active = was_capturing;
            } else {
                send_nak(0x02);
            }
            break;

        case CMD_SET_MASK:
            if (param_len >= 6) {
                uint8_t mask_num = params[0];
                uint32_t mask = params[1] | (params[2] << 8) |
                               (params[3] << 16) | (params[4] << 24);
                bool extended = params[5] != 0;

                // Save current mode to restore after
                mcp2515_mode_t prev_mode = mcp2515_get_mode();

                // Enter config mode, set mask, restore mode
                if (mcp2515_set_mode(MCP2515_MODE_CONFIG)) {
                    if (mcp2515_set_mask(mask_num, mask, extended)) {
                        mcp2515_set_mode(prev_mode);
                        send_ack();
                    } else {
                        mcp2515_set_mode(prev_mode);
                        send_nak(0x06);  // Invalid mask
                    }
                } else {
                    send_nak(0x03);  // Mode change failed
                }
            } else {
                send_nak(0x02);
            }
            break;

        case CMD_SET_ONESHOT:
            if (param_len >= 1) {
                mcp2515_set_oneshot_mode(params[0] != 0);
                send_ack();
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

        case CMD_RESET_CAN:
            // Reset CAN controller and restore configuration
            // Use this to recover from bus-off or clear TEC/REC
            {
                bool was_capturing = g_capture_active;
                g_capture_active = false;

                if (mcp2515_reset_and_restore()) {
                    send_ack();
                } else {
                    send_nak(0x07);  // Reset failed
                }

                g_capture_active = was_capturing;
            }
            break;

        default:
            send_nak(0xFF);  // Unknown command
            break;
    }
}

void commands_init(void) {
    cmd_len = 0;
    cmd_expected_len = 0;
    parse_state = WAIT_STX;
}

void commands_process(void) {
    uint8_t rx_buf[64];
    int rx_len = usb_read(rx_buf, sizeof(rx_buf));

    for (int i = 0; i < rx_len; i++) {
        uint8_t byte = rx_buf[i];

        switch (parse_state) {
            case WAIT_STX:
                if (byte == 0x02) {
                    cmd_len = 0;
                    parse_state = READ_OPCODE;
                }
                break;

            case READ_OPCODE:
                cmd_buffer[cmd_len++] = byte;
                parse_state = READ_LEN;
                break;

            case READ_LEN:
                cmd_buffer[cmd_len++] = byte;
                cmd_expected_len = byte;
                if (cmd_expected_len == 0) {
                    parse_state = WAIT_ETX;
                } else if (cmd_expected_len > CMD_BUFFER_SIZE - 2) {
                    // Payload too large - NAK and discard until ETX to resync
                    send_nak(0x08);  // Error code: payload too large
                    parse_state = DISCARD_UNTIL_ETX;
                } else {
                    parse_state = READ_PAYLOAD;
                }
                break;

            case READ_PAYLOAD:
                if (cmd_len < CMD_BUFFER_SIZE) {
                    cmd_buffer[cmd_len++] = byte;
                }
                // Check if we've read all expected payload bytes
                if (cmd_len >= cmd_expected_len + 2) {
                    parse_state = WAIT_ETX;
                }
                break;

            case WAIT_ETX:
                if (byte == 0x03) {
                    // Valid packet - process command
                    uint8_t opcode = cmd_buffer[0];
                    uint8_t param_len = cmd_buffer[1];
                    handle_command(opcode, &cmd_buffer[2], param_len);
                }
                // Reset for next packet (even if ETX was wrong)
                parse_state = WAIT_STX;
                cmd_len = 0;
                break;

            case DISCARD_UNTIL_ETX:
                // Discard bytes until ETX arrives, then resync
                // This handles oversized payloads without getting stuck
                if (byte == 0x03) {
                    parse_state = WAIT_STX;
                    cmd_len = 0;
                }
                break;
        }
    }
}

void commands_get_status(device_status_t *status) {
    const can_config_t *cfg = mcp2515_get_config();

    // Read actual CNF registers and calculate speed
    can_registers_t regs;
    mcp2515_get_registers(&regs);
    uint32_t speed = calculate_speed_from_cnf(regs.cnf1, regs.cnf2, regs.cnf3);

    status->protocol_version = PROTOCOL_VERSION;
    status->mode = (uint8_t)cfg->mode;
    status->can_speed = speed;  // From actual registers
    status->capture_active = g_capture_active;  // Shared volatile state
    status->error_flags = regs.eflg;
    status->rx_frame_count = usb_get_rx_count();        // Frames queued (Core 1)
    status->tx_frame_count = usb_get_tx_to_host_count(); // Frames sent to host (Core 0)
}
