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
| 0x10 | START_CAPTURE | - | Start CAN frame capture |
| 0x11 | STOP_CAPTURE | - | Stop CAN frame capture |
| 0x20 | SET_SPEED | speed:u32 | Set CAN bus speed (bps) |
| 0x21 | SET_FILTER | filter_num:u8, id:u32, mask:u32, extended:u8 | Set acceptance filter |
| 0x22 | CLEAR_FILTERS | - | Clear all filters |
| 0x23 | SET_MODE | mode:u8 | Set MCP2515 mode |
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
| 11 | reserved | 1 | Reserved |

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

## Error Codes (NAK)

| Code | Description |
|------|-------------|
| 0x01 | Invalid speed |
| 0x02 | Invalid parameters |
| 0x03 | Mode change failed |
| 0x04 | Transmit failed |
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
