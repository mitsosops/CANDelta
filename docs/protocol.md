# CANDelta Wire Protocol

Binary protocol over USB CDC (virtual COM port) at 115200 baud.

## Packet Format

All packets use length-prefixed framing with STX (0x02) and ETX (0x03):

```
[STX] [opcode:u8] [length:u8] [payload...] [ETX]
0x02                                        0x03
```

**Important:** The `length` byte specifies the payload size. This allows 0x02/0x03 bytes
to appear in the payload without breaking framing (length-prefixed parsing).

## Commands (Host -> Device)

| Opcode | Name | Parameters | Description |
|--------|------|------------|-------------|
| 0x01 | PING | - | Verify connection |
| 0x04 | GET_VERSION | - | Get firmware version |
| 0x05 | GET_STATUS | - | Get device status |
| 0x06 | DEBUG | - | Get debug info (buffer state, MCP2515 registers) |
| 0x07 | GET_PERF_STATS | - | Get performance statistics |
| 0x08 | GET_DEVICE_ID | - | Get unique device ID (8 bytes from RP2040 flash) |
| 0x09 | GET_ERROR_COUNTERS | - | Get CAN 2.0B error counters (TEC/REC/state) |
| 0x0A | LIST_COMMANDS | - | List available commands with parameter counts |
| 0x0B | GET_CONFIG | - | Get full device configuration |
| 0x0C | GET_REGISTERS | - | Get raw MCP2515 register values |
| 0x10 | START_CAPTURE | - | Start CAN frame capture |
| 0x11 | STOP_CAPTURE | - | Stop CAN frame capture |
| 0x20 | SET_SPEED | speed:u32 | Set CAN bus speed (bps) |
| 0x21 | SET_FILTER | filter_num:u8, id:u32, extended:u8 | Set acceptance filter (0-5) |
| 0x22 | CLEAR_FILTERS | - | Clear all filters (accept all mode) |
| 0x23 | SET_MODE | mode:u8 | Set MCP2515 mode |
| 0x24 | SET_TIMING | cnf1:u8, cnf2:u8, cnf3:u8 | Set custom bit timing (advanced) |
| 0x25 | SET_MASK | mask_num:u8, mask:u32, extended:u8 | Set acceptance mask (0-1) |
| 0x26 | SET_ONESHOT | enabled:u8 | Enable/disable one-shot TX mode |
| 0x27 | RESET_CAN | - | Reset CAN controller and restore config (clears TEC/REC) |
| 0x30 | TRANSMIT_FRAME | id:u32, flags:u8, dlc:u8, data:u8[0-8] | Transmit a CAN frame |

**Note:** Opcodes 0x02 and 0x03 are avoided as they conflict with STX/ETX framing bytes.

## Responses (Device -> Host)

| Code | Name | Payload | Description |
|------|------|---------|-------------|
| 0x80 | ACK | - | Command succeeded |
| 0x81 | NAK | error_code:u8 | Command failed |
| 0x82 | VERSION | protocol:u8, major:u8, minor:u8, patch:u8 | Version info |
| 0x83 | STATUS | see below | Device status |
| 0x84 | CAN_FRAME | see below | Captured CAN frame (async) |
| 0x85 | DEBUG | see below | Debug info |
| 0x86 | PERF_STATS | see below | Performance statistics |
| 0x87 | DEVICE_ID | id:u8[8] | Unique device ID (factory-programmed) |
| 0x88 | ERROR_COUNTERS | see below | CAN 2.0B error counters |
| 0x89 | COMMAND_LIST | see below | List of available commands |
| 0x8A | CONFIG | see below | Full device configuration |
| 0x8B | REGISTERS | see below | Raw MCP2515 register values |

## CAN Frame Format (0x84)

Streamed asynchronously during capture:

```
[STX] [0x84] [length:u8] [timestamp:u64] [id:u32] [flags:u8] [dlc:u8] [data:u8[dlc]] [ETX]
```

| Field | Size | Description |
|-------|------|-------------|
| timestamp | 8 bytes | Microseconds since device boot (little-endian) |
| id | 4 bytes | CAN ID (little-endian) |
| flags | 1 byte | bit 0 = extended ID, bit 1 = RTR |
| dlc | 1 byte | Data length (0-8) |
| data | 0-8 bytes | Frame payload |

**Payload length:** 14 + dlc bytes

## Status Payload (0x83)

16 bytes:

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | protocol_version | 1 | Protocol version |
| 1 | mode | 1 | Current MCP2515 mode |
| 2 | can_speed | 4 | CAN bus speed in bps (LE) |
| 6 | capture_active | 1 | 1 if capturing, 0 otherwise |
| 7 | error_flags | 1 | MCP2515 error flags |
| 8 | rx_frame_count | 4 | Frames received (LE) |
| 12 | tx_frame_count | 4 | Frames transmitted to host (LE) |

## Performance Stats Payload (0x86)

13 bytes:

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | frames_per_second | 4 | Current FPS (LE) |
| 4 | peak_fps | 4 | Maximum FPS observed since boot (LE) |
| 8 | dropped_frames | 4 | Frames lost due to buffer full (LE) |
| 12 | buffer_utilization | 1 | Buffer fill percentage (0-100) |

**Usage:** Poll this command every 500ms-1s for live performance monitoring.
Stats are tracked internally at all times with negligible overhead.

## Debug Payload (0x85)

12 bytes:

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | buffer_head | 2 | Ring buffer head index (LE) |
| 2 | buffer_tail | 2 | Ring buffer tail index (LE) |
| 4 | rx_count | 1 | Frames queued (low byte) |
| 5 | tx_count | 1 | Frames sent to host (low byte) |
| 6 | capture_active | 1 | Capture state |
| 7 | canintf | 1 | MCP2515 CANINTF register |
| 8 | canstat | 1 | MCP2515 CANSTAT register |
| 9 | eflg | 1 | MCP2515 error flags |
| 10 | cnf1 | 1 | MCP2515 CNF1 register |
| 11 | txb0ctrl | 1 | MCP2515 TXB0CTRL register (TXREQ, TXERR, ABTF) |

## Device ID Payload (0x87)

8 bytes: The RP2040's factory-programmed unique ID from flash.

This ID is:
- Unique per device (factory-burned)
- Persistent across power cycles and reflashing
- Useful for identifying specific CANDelta devices

The same ID is also used as the USB serial number, allowing the desktop app to
match connected USB devices with their protocol-reported IDs.

## Error Counters Payload (0x88)

3 bytes:

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | tec | 1 | Transmit Error Counter (0-255) |
| 1 | rec | 1 | Receive Error Counter (0-255) |
| 2 | error_state | 1 | CAN error state (see below) |

**Error States:**

| Value | State | Description |
|-------|-------|-------------|
| 0 | ERROR_ACTIVE | Normal operation (TEC/REC < 96) |
| 1 | ERROR_WARNING | Warning threshold (TEC or REC >= 96) |
| 2 | ERROR_PASSIVE | Error passive (TEC or REC >= 128) |
| 3 | BUS_OFF | Bus-off (TEC >= 256) |

## Command List Payload (0x89)

Variable length: Array of (opcode:u8, param_count:u8) pairs.

Each pair indicates:
- `opcode`: The command opcode
- `param_count`: Number of logical parameters (not byte count)

**Example response:**
```
02 89 24 01 00 04 00 05 00 ... 20 01 21 03 ... 03
      |     |     |     |       |     |
      +- len +- PING,0  +- ...  +- SET_SPEED,1
                                      +- SET_FILTER,3
```

**Note:** Multi-byte parameters (e.g., u32) count as 1 parameter.
TRANSMIT_FRAME has variable data length after the 3 fixed parameters.

## Config Payload (0x8A)

49 bytes: Full device configuration (intended settings).

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | speed_bps | 4 | CAN speed in bps (LE), 0 if custom timing |
| 4 | cnf1 | 1 | Timing register CNF1 |
| 5 | cnf2 | 1 | Timing register CNF2 |
| 6 | cnf3 | 1 | Timing register CNF3 |
| 7 | mode | 1 | Operating mode (0-4) |
| 8 | flags | 1 | Configuration flags (see below) |
| 9 | filters | 30 | 6 filters × 5 bytes each |
| 39 | masks | 10 | 2 masks × 5 bytes each |

**Flags byte:**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | custom_timing | 1 if SET_TIMING was used instead of SET_SPEED |
| 1 | filters_active | 1 if filtering enabled (vs accept-all) |
| 2 | rollover_enabled | 1 if RXB0 overflow rolls to RXB1 |
| 3 | oneshot_enabled | 1 if one-shot TX mode enabled |
| 4 | capture_active | 1 if capture is running |

**Filter format (5 bytes each):**

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | id | 4 | Filter ID (LE) |
| 4 | flags | 1 | bit0=enabled, bit1=extended |

**Mask format (5 bytes each):**

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | mask | 4 | Mask value (LE) |
| 4 | flags | 1 | bit0=enabled, bit1=extended |

## Registers Payload (0x8B)

15 bytes: Raw MCP2515 register values (actual hardware state).

| Offset | Field | Size | Description |
|--------|-------|------|-------------|
| 0 | cnf1 | 1 | CNF1 register (SJW + BRP) |
| 1 | cnf2 | 1 | CNF2 register (BTLMODE + SAM + PHSEG1 + PRSEG) |
| 2 | cnf3 | 1 | CNF3 register (SOF + WAKFIL + PHSEG2) |
| 3 | canstat | 1 | CANSTAT register (mode in bits 7:5) |
| 4 | canctrl | 1 | CANCTRL register (OSM in bit 3) |
| 5 | eflg | 1 | Error flags register |
| 6 | canintf | 1 | Interrupt flags register |
| 7 | tec | 1 | Transmit Error Counter |
| 8 | rec | 1 | Receive Error Counter |
| 9 | txb0ctrl | 1 | TX buffer 0 control |
| 10 | txb1ctrl | 1 | TX buffer 1 control |
| 11 | txb2ctrl | 1 | TX buffer 2 control |
| 12 | rxb0ctrl | 1 | RX buffer 0 control (filter mode in bits 6:5) |
| 13 | rxb1ctrl | 1 | RX buffer 1 control |
| 14 | mismatch_flags | 1 | Config vs register mismatches (see below) |

**Mismatch flags:**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | mode_mismatch | Actual mode differs from intended |
| 1 | timing_mismatch | CNF registers differ from config |
| 2 | oneshot_mismatch | OSM bit differs from config |
| 3 | filter_mode_mismatch | RXM bits differ from filters_active |

Use this to detect if the MCP2515 state has diverged from the intended
configuration (e.g., after an unexpected reset or communication error).

## CAN Speeds

| Value | Speed |
|-------|-------|
| 125000 | 125 kbps |
| 250000 | 250 kbps |
| 500000 | 500 kbps |
| 1000000 | 1 Mbps |

## MCP2515 Modes

| Value | Mode | Description |
|-------|------|-------------|
| 0 | Normal | Standard operation |
| 1 | Sleep | Low power mode |
| 2 | Loopback | TX frames loop back to RX (self-test) |
| 3 | Listen-only | Receive only, no ACKs sent |
| 4 | Configuration | Register access mode |

## Automatic Bus-Off Recovery

The firmware automatically monitors for CAN bus errors via the MCP2515's error interrupt.
When bus-off is detected (TEC >= 256), the firmware automatically:

1. Resets the MCP2515 controller (clears TEC/REC)
2. Restores all saved configuration (timing, filters, masks, mode)
3. Flashes the red LED (500ms) to indicate recovery
4. Resumes normal operation

This is essential for request-response protocols (UDS, KWP2000, OBD2) where
automatic recovery ensures communication can continue after transient errors.

**Error LED Indication:**
- Bus-off: 500ms red flash (auto-recovery triggered)
- Warning/Passive: 100ms red flash

**Manual Recovery:**
Use `CMD_RESET_CAN` (0x27) to manually trigger reset and restore if needed.

## Error Codes (NAK)

| Code | Description |
|------|-------------|
| 0x01 | Invalid speed |
| 0x02 | Invalid parameters |
| 0x03 | Mode change failed |
| 0x04 | Transmit failed |
| 0x05 | Timing configuration failed |
| 0x06 | Invalid filter or mask |
| 0x07 | CAN reset failed |
| 0xFF | Unknown command |

## Example: Get Performance Stats

**Request:**
```
02 07 00 03
```

**Response (example: 400 fps, peak 419, 0 dropped, 5% buffer):**
```
02 86 0D 90 01 00 00 A3 01 00 00 00 00 00 00 05 03
      |  |           |           |           |
      |  +- fps=400  +- peak=419 +- drop=0   +- util=5%
      +- length=13
```
