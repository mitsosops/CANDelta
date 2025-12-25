# CANDelta Wire Protocol

Binary protocol over USB CDC (virtual COM port) at 115200 baud.

## Packet Format

All packets are framed with STX (0x02) and ETX (0x03):

```
[STX] [payload...] [ETX]
```

## Commands (Host → Device)

| Opcode | Name | Parameters | Description |
|--------|------|------------|-------------|
| 0x01 | PING | - | Verify connection |
| 0x02 | GET_VERSION | - | Get firmware version |
| 0x03 | GET_STATUS | - | Get device status |
| 0x10 | START_CAPTURE | - | Start CAN frame capture |
| 0x11 | STOP_CAPTURE | - | Stop CAN frame capture |
| 0x20 | SET_SPEED | speed:u32 | Set CAN bus speed (bps) |
| 0x21 | SET_FILTER | filter_num:u8, id:u32, mask:u32, extended:u8 | Set acceptance filter |
| 0x22 | CLEAR_FILTERS | - | Clear all filters |
| 0x23 | SET_MODE | mode:u8 | Set MCP2515 mode |
| 0x30 | TRANSMIT_FRAME | id:u32, flags:u8, dlc:u8, data:u8[dlc] | Transmit a CAN frame |

Command packet format:
```
[STX] [opcode:u8] [param_len:u8] [params...] [ETX]
```

## Responses (Device → Host)

| Code | Name | Payload | Description |
|------|------|---------|-------------|
| 0x80 | ACK | - | Command succeeded |
| 0x81 | NAK | error_code:u8 | Command failed |
| 0x82 | VERSION | protocol:u8, major:u8, minor:u8, patch:u8 | Version info |
| 0x83 | STATUS | see below | Device status |
| 0x84 | CAN_FRAME | see below | Captured CAN frame |

Response packet format:
```
[STX] [code:u8] [payload_len:u8] [payload...] [ETX]
```

## CAN Frame Format (Async)

Streamed during capture (no response code prefix):
```
[STX] [timestamp:u64] [id:u32] [flags:u8] [dlc:u8] [data:u8[dlc]] [ETX]
```

- **timestamp**: Microseconds since device boot (little-endian)
- **id**: CAN ID (little-endian)
- **flags**: bit 0 = extended ID, bit 1 = RTR
- **dlc**: Data length (0-8)
- **data**: Frame payload

## Status Payload

```
protocol_version: u8
mode: u8
can_speed: u32 (LE)
capture_active: u8
error_flags: u8
rx_frame_count: u32 (LE)
tx_frame_count: u32 (LE)
```

## CAN Speeds

| Value | Speed |
|-------|-------|
| 125000 | 125 kbps |
| 250000 | 250 kbps |
| 500000 | 500 kbps |
| 1000000 | 1 Mbps |

## MCP2515 Modes

| Value | Mode |
|-------|------|
| 0 | Normal |
| 1 | Sleep |
| 2 | Loopback |
| 3 | Listen-only |
| 4 | Configuration |

## Error Codes (NAK)

| Code | Description |
|------|-------------|
| 0x01 | Invalid speed |
| 0x02 | Invalid parameters |
| 0x03 | Mode change failed |
| 0x04 | Transmit failed |
| 0xFF | Unknown command |
