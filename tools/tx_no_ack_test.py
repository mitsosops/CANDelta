#!/usr/bin/env python3
"""
Test: What happens when TX sends frames with no receiver to ACK?

This helps us understand the CAN error behavior.
Run this with the CANDelta UNPOWERED or DISCONNECTED from the bus.
"""

import sys
import time
import serial


def slcan_init(ser, bitrate=500000):
    """Initialize SLCAN adapter"""
    ser.write(b'C\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    bitrate_map = {500000: 'S6', 250000: 'S5', 125000: 'S4'}
    cmd = bitrate_map.get(bitrate, 'S6')
    ser.write(f'{cmd}\r'.encode())
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(b'O\r')
    time.sleep(0.2)
    ser.read(ser.in_waiting)


def slcan_send(ser, can_id, data):
    """Send a CAN frame and return response"""
    cmd = f't{can_id:03X}{len(data):01X}'
    for byte in data:
        cmd += f'{byte:02X}'
    cmd += '\r'
    ser.write(cmd.encode())
    ser.flush()
    time.sleep(0.01)
    return ser.read(ser.in_waiting)


def main():
    if len(sys.argv) < 2:
        print("Usage: tx_no_ack_test.py <COM_PORT>")
        print("Run with CANDelta UNPOWERED/DISCONNECTED to test no-ACK behavior")
        return

    port = sys.argv[1]

    print("=" * 60)
    print("TX NO-ACK TEST")
    print("=" * 60)
    print(f"Port: {port}")
    print("Make sure CANDelta is UNPOWERED or DISCONNECTED from CAN bus!")
    print("=" * 60)
    input("Press Enter to continue...")

    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening {port}: {e}")
        return

    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print("\nInitializing SLCAN adapter at 500kbps...")
    slcan_init(ser, 500000)

    print("\nSending 20 frames with no receiver (expecting ACK errors)...")
    print("Response codes: CR (\\r) = OK, BEL (0x07) = Error\n")

    ok_count = 0
    error_count = 0
    no_response = 0

    for i in range(20):
        data = [i & 0xFF, 0xAA, 0xBB, 0xCC]
        resp = slcan_send(ser, 0x123, data)

        resp_hex = resp.hex() if resp else "(none)"

        if b'\r' in resp or b'z\r' in resp or b'Z\r' in resp:
            status = "OK"
            ok_count += 1
        elif b'\x07' in resp:
            status = "ERROR"
            error_count += 1
        elif not resp:
            status = "NO RESPONSE"
            no_response += 1
        else:
            status = f"UNKNOWN: {resp_hex}"

        print(f"  Frame {i+1:2d}: {status} (raw: {resp_hex})")
        time.sleep(0.05)  # 50ms between frames

    # Close channel
    ser.write(b'C\r')
    ser.close()

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"  OK responses:       {ok_count}")
    print(f"  Error responses:    {error_count}")
    print(f"  No response:        {no_response}")
    print()

    if error_count == 0 and ok_count > 0:
        print("OBSERVATION: Adapter reports OK even without ACK!")
        print("This suggests the adapter either:")
        print("  1. Has one-shot mode (doesn't wait for ACK/retry)")
        print("  2. Doesn't report ACK errors to host")
        print("  3. Has auto-recovery from errors")
    elif error_count > 0:
        print(f"OBSERVATION: Adapter detected {error_count} errors (as expected)")
    else:
        print("OBSERVATION: No responses received - check adapter connection")

    print("=" * 60)


if __name__ == "__main__":
    main()
