#!/usr/bin/env python3
"""TX verification tool for CANDelta using loopback mode"""

import sys
import time
import serial
import struct

# Protocol constants
STX = 0x02
ETX = 0x03

# Commands
CMD_PING = 0x01
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_SET_MODE = 0x23
CMD_TRANSMIT_FRAME = 0x30

# Responses
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_CAN_FRAME = 0x84

# MCP2515 modes (enum values, not register values)
MODE_NORMAL = 0      # MCP2515_MODE_NORMAL
MODE_LOOPBACK = 2    # MCP2515_MODE_LOOPBACK


def send_cmd(ser, opcode, params=None):
    """Send command and wait for response"""
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.1)
    return ser.read(ser.in_waiting)


CMD_DEBUG = 0x06
RSP_DEBUG = 0x85


def get_debug_info(ser):
    """Get MCP2515 debug info"""
    response = send_cmd(ser, CMD_DEBUG)
    if RSP_DEBUG in response:
        idx = list(response).index(RSP_DEBUG)
        if idx + 13 <= len(response):
            payload = response[idx + 2 : idx + 14]
            return {
                "head": payload[0] | (payload[1] << 8),
                "tail": payload[2] | (payload[3] << 8),
                "rx_count": payload[4],
                "tx_count": payload[5],
                "capture": payload[6],
                "canintf": payload[7],
                "canstat": payload[8],
                "eflg": payload[9],
                "cnf1": payload[10],
            }
    return None


def set_loopback_mode(ser):
    """Put MCP2515 into loopback mode"""
    response = send_cmd(ser, CMD_SET_MODE, [MODE_LOOPBACK])
    if RSP_ACK in response:
        return True
    return False


def set_normal_mode(ser):
    """Put MCP2515 into normal mode"""
    response = send_cmd(ser, CMD_SET_MODE, [MODE_NORMAL])
    if RSP_ACK in response:
        return True
    return False


def transmit_frame_raw(ser, can_id, data, extended=False, rtr=False):
    """Transmit a CAN frame and return all response data"""
    params = []
    # CAN ID (4 bytes, little-endian)
    params.append(can_id & 0xFF)
    params.append((can_id >> 8) & 0xFF)
    params.append((can_id >> 16) & 0xFF)
    params.append((can_id >> 24) & 0xFF)
    # Flags: bit 0 = extended, bit 1 = RTR
    flags = (0x01 if extended else 0x00) | (0x02 if rtr else 0x00)
    params.append(flags)
    # DLC
    params.append(len(data))
    # Data
    params.extend(data)

    packet = bytes([STX, CMD_TRANSMIT_FRAME, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()

    # Short wait for ACK only
    time.sleep(0.05)
    ack_response = ser.read(ser.in_waiting)

    if RSP_ACK not in ack_response:
        return None, None

    # Wait longer for loopback frame
    time.sleep(0.3)
    frame_response = ser.read(ser.in_waiting)

    return ack_response, frame_response


def transmit_frame(ser, can_id, data, extended=False, rtr=False):
    """Transmit a CAN frame (simple version for non-loopback tests)"""
    ack, _ = transmit_frame_raw(ser, can_id, data, extended, rtr)
    return ack is not None


def parse_can_frame(data):
    """Parse a received CAN frame from response data"""
    # Find RSP_CAN_FRAME in data
    if RSP_CAN_FRAME not in data:
        return None

    idx = list(data).index(RSP_CAN_FRAME)
    if idx + 2 >= len(data):
        return None

    payload_len = data[idx + 1]
    if idx + 2 + payload_len > len(data):
        return None

    payload = data[idx + 2 : idx + 2 + payload_len]
    if len(payload) < 14:
        return None

    # Parse: timestamp(8) + id(4) + flags(1) + dlc(1) + data(0-8)
    timestamp = struct.unpack("<Q", payload[0:8])[0]
    can_id = struct.unpack("<I", payload[8:12])[0]
    flags = payload[12]
    dlc = payload[13]
    frame_data = list(payload[14 : 14 + dlc])

    return {
        "timestamp": timestamp,
        "id": can_id,
        "extended": (flags & 0x01) != 0,
        "rtr": (flags & 0x02) != 0,
        "dlc": dlc,
        "data": frame_data,
    }


def run_loopback_test(ser, test_name, can_id, data, extended=False, rtr=False):
    """Run a single loopback test"""
    print(f"\n  Test: {test_name}")
    id_str = f"0x{can_id:08X}" if extended else f"0x{can_id:03X}"
    print(f"    TX: ID={id_str} DLC={len(data)} Data={data}")

    # Transmit frame and get loopback response
    ack_response, frame_response = transmit_frame_raw(ser, can_id, data, extended, rtr)

    if ack_response is None:
        print("    [FAIL] TX failed (NAK received)")
        return False

    # Combine all response data to find the CAN frame
    response = ack_response + (frame_response or b"")

    # Parse received frame
    frame = parse_can_frame(response)
    if frame is None:
        print("    [FAIL] No frame received")
        return False

    rx_id_str = f"0x{frame['id']:08X}" if frame['extended'] else f"0x{frame['id']:03X}"
    print(f"    RX: ID={rx_id_str} DLC={frame['dlc']} Data={frame['data']}")

    # Verify frame matches
    if frame["id"] != can_id:
        print(f"    [FAIL] ID mismatch: expected 0x{can_id:X}, got 0x{frame['id']:X}")
        return False

    if frame["extended"] != extended:
        print(f"    [FAIL] Extended flag mismatch")
        return False

    if frame["dlc"] != len(data):
        print(f"    [FAIL] DLC mismatch: expected {len(data)}, got {frame['dlc']}")
        return False

    if not rtr and frame["data"] != data:
        print(f"    [FAIL] Data mismatch")
        return False

    print("    [PASS]")
    return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    print(f"CANDelta TX Verification Tool")
    print(f"=" * 50)
    print(f"Port: {port}")

    # Connect
    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)  # Flush

    # Ping test
    print("\nPing test...")
    response = send_cmd(ser, CMD_PING)
    if RSP_ACK not in response:
        print("[FAIL] Device not responding")
        ser.close()
        return 1
    print("[PASS] Device responding")

    # Enter loopback mode
    print("\nEntering loopback mode...")
    if not set_loopback_mode(ser):
        print("[FAIL] Could not set loopback mode")
        ser.close()
        return 1
    print("[PASS] Loopback mode enabled")

    # Start capture to receive loopback frames
    send_cmd(ser, CMD_START_CAPTURE)
    time.sleep(0.1)

    # Run tests
    print("\n" + "=" * 50)
    print("LOOPBACK TESTS")
    print("=" * 50)

    passed = 0
    failed = 0

    # Test 1: Standard frame with data
    if run_loopback_test(ser, "Standard frame (11-bit ID)", 0x123, [0x11, 0x22, 0x33, 0x44]):
        passed += 1
    else:
        failed += 1

    # Test 2: Standard frame with max data
    if run_loopback_test(ser, "Standard frame (8 bytes)", 0x7FF, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]):
        passed += 1
    else:
        failed += 1

    # Test 3: Standard frame with no data
    if run_loopback_test(ser, "Standard frame (0 bytes)", 0x100, []):
        passed += 1
    else:
        failed += 1

    # Test 4: Extended frame
    if run_loopback_test(ser, "Extended frame (29-bit ID)", 0x18DAF110, [0xAA, 0xBB, 0xCC], extended=True):
        passed += 1
    else:
        failed += 1

    # Test 5: UDS-style request (common for diagnostic protocols)
    if run_loopback_test(ser, "UDS-style frame (0x7DF)", 0x7DF, [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]):
        passed += 1
    else:
        failed += 1

    # Test 6: OBD2-style request
    if run_loopback_test(ser, "OBD2-style frame (0x7E0)", 0x7E0, [0x02, 0x09, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]):
        passed += 1
    else:
        failed += 1

    # Stop capture
    send_cmd(ser, CMD_STOP_CAPTURE)

    # Restore normal mode
    print("\nRestoring normal mode...")
    set_normal_mode(ser)
    print("Done")

    # Summary
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")

    if failed == 0:
        print("\n  [ALL TESTS PASSED]")
        print("  TX functionality verified for diagnostic protocols")
    else:
        print(f"\n  [SOME TESTS FAILED]")

    ser.close()
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
