#!/usr/bin/env python3
"""
CANDelta Firmware Test Script
Tests basic communication with the RP2040 CAN analyzer.

Usage: python test_firmware.py COM3
"""

import sys
import time
import serial
import serial.tools.list_ports

# Protocol constants
STX = 0x02
ETX = 0x03

# Commands (avoid 0x02/0x03 - they conflict with STX/ETX framing)
CMD_PING = 0x01
CMD_GET_VERSION = 0x04
CMD_GET_STATUS = 0x05
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11

# Responses
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_VERSION = 0x82
RSP_STATUS = 0x83


def list_ports():
    """List available COM ports."""
    ports = serial.tools.list_ports.comports()
    print("\nAvailable COM ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return ports


DEBUG = True

def send_command(ser, opcode, params=b''):
    """Send a command and return the response."""
    packet = bytes([STX, opcode, len(params)]) + params + bytes([ETX])
    if DEBUG:
        print(f"    TX: {packet.hex()}")
    ser.write(packet)
    ser.flush()

    # Wait for response
    time.sleep(0.1)

    # Read response
    response = b''
    raw_received = b''
    start_time = time.time()
    in_packet = False

    while time.time() - start_time < 1.0:  # 1 second timeout
        if ser.in_waiting:
            byte = ser.read(1)
            raw_received += byte
            if byte[0] == STX:
                in_packet = True
                response = b''
            elif byte[0] == ETX and in_packet:
                if DEBUG:
                    print(f"    RX: {raw_received.hex()}")
                return response
            elif in_packet:
                response += byte
        else:
            time.sleep(0.01)

    if DEBUG and raw_received:
        print(f"    RX (incomplete): {raw_received.hex()}")
    # Check if more data arrives after timeout
    time.sleep(0.2)
    if ser.in_waiting:
        extra = ser.read(ser.in_waiting)
        print(f"    RX (late): {extra.hex()}")
    return None


def test_ping(ser):
    """Test PING command."""
    print("\n[TEST] PING...")
    response = send_command(ser, CMD_PING)

    if response and len(response) >= 2 and response[0] == RSP_ACK:
        print("  [OK] PING successful - device is responding!")
        return True
    else:
        print(f"  [FAIL] PING failed - response: {response}")
        return False


def test_version(ser):
    """Test GET_VERSION command."""
    print("\n[TEST] GET_VERSION...")
    response = send_command(ser, CMD_GET_VERSION)

    if response and len(response) >= 6 and response[0] == RSP_VERSION:
        protocol = response[2]
        major = response[3]
        minor = response[4]
        patch = response[5]
        print(f"  [OK] Firmware version: {major}.{minor}.{patch} (protocol v{protocol})")
        return True
    else:
        print(f"  [FAIL] GET_VERSION failed - response: {response}")
        return False


def test_status(ser):
    """Test GET_STATUS command."""
    print("\n[TEST] GET_STATUS...")
    response = send_command(ser, CMD_GET_STATUS)

    if response and len(response) >= 2 and response[0] == RSP_STATUS:
        payload_len = response[1]
        print(f"  [OK] Status received ({payload_len} bytes)")
        if len(response) >= 18:  # 2 header + 16 data
            protocol = response[2]
            mode = response[3]
            speed = int.from_bytes(response[4:8], 'little')
            capture = response[8]
            error_flags = response[9]
            rx_count = int.from_bytes(response[10:14], 'little')
            tx_count = int.from_bytes(response[14:18], 'little')
            print(f"    Protocol: {protocol}")
            print(f"    Mode: {mode}")
            print(f"    CAN Speed: {speed} bps")
            print(f"    Capture active: {bool(capture)}")
            print(f"    Error flags: 0x{error_flags:02X}")
            print(f"    RX count: {rx_count}")
            print(f"    TX count: {tx_count}")
        return True
    else:
        print(f"  [FAIL] GET_STATUS failed - response: {response}")
        return False


def test_capture_toggle(ser):
    """Test START/STOP capture commands."""
    print("\n[TEST] START_CAPTURE...")
    response = send_command(ser, CMD_START_CAPTURE)

    if response and len(response) >= 2 and response[0] == RSP_ACK:
        print("  [OK] Capture started")
    else:
        print(f"  [FAIL] START_CAPTURE failed - response: {response}")
        return False

    time.sleep(0.5)

    print("\n[TEST] STOP_CAPTURE...")
    response = send_command(ser, CMD_STOP_CAPTURE)

    if response and len(response) >= 2 and response[0] == RSP_ACK:
        print("  [OK] Capture stopped")
        return True
    else:
        print(f"  [FAIL] STOP_CAPTURE failed - response: {response}")
        return False


def main():
    print("=" * 50)
    print("CANDelta Firmware Test")
    print("=" * 50)

    # Get COM port
    if len(sys.argv) < 2:
        ports = list_ports()
        if not ports:
            print("\nNo COM ports found. Is the device connected?")
            return 1
        print("\nUsage: python test_firmware.py <COM_PORT>")
        print("Example: python test_firmware.py COM3")
        return 1

    port = sys.argv[1]
    print(f"\nConnecting to {port}...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1
        )
        ser.dtr = True
        ser.rts = True
        time.sleep(0.5)  # Wait for device to initialize

        print(f"  Connected to {port}")

        # Run tests
        results = []
        results.append(("PING", test_ping(ser)))
        results.append(("VERSION", test_version(ser)))
        results.append(("STATUS", test_status(ser)))
        results.append(("CAPTURE", test_capture_toggle(ser)))

        ser.close()

        # Summary
        print("\n" + "=" * 50)
        print("Test Summary:")
        print("=" * 50)
        passed = sum(1 for _, r in results if r)
        for name, result in results:
            status = "PASS" if result else "FAIL"
            print(f"  {name}: {status}")
        print(f"\nTotal: {passed}/{len(results)} tests passed")

        return 0 if passed == len(results) else 1

    except serial.SerialException as e:
        print(f"\n  [ERROR] {e}")
        print("\nTroubleshooting:")
        print("  1. Check the device is connected and flashed")
        print("  2. Check Device Manager for the correct COM port")
        print("  3. Close any other programs using the port")
        return 1


if __name__ == "__main__":
    sys.exit(main())
