#!/usr/bin/env python3
"""
CANDelta CAN Debug - Check for errors and bus status
"""

import sys
import time
import serial

STX = 0x02
ETX = 0x03
CMD_GET_STATUS = 0x05
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
RSP_STATUS = 0x83
RSP_ACK = 0x80


def send_command(ser, opcode):
    packet = bytes([STX, opcode, 0, ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.1)
    return ser.read(ser.in_waiting)


def parse_status(data):
    # Find the packet
    if STX in data and RSP_STATUS in data:
        idx = data.index(RSP_STATUS)
        if idx + 17 <= len(data):
            payload = data[idx+2:idx+18]
            error_flags = payload[5]
            rx_count = int.from_bytes(payload[6:10], 'little')

            print(f"  RX frame count: {rx_count}")
            print(f"  Error flags: 0x{error_flags:02X}")

            # Decode MCP2515 error flags (EFLG register)
            if error_flags & 0x80: print("    - RX1OVR: RX Buffer 1 Overflow")
            if error_flags & 0x40: print("    - RX0OVR: RX Buffer 0 Overflow")
            if error_flags & 0x20: print("    - TXBO: Bus-Off")
            if error_flags & 0x10: print("    - TXEP: TX Error-Passive")
            if error_flags & 0x08: print("    - RXEP: RX Error-Passive")
            if error_flags & 0x04: print("    - TXWAR: TX Warning")
            if error_flags & 0x02: print("    - RXWAR: RX Warning")
            if error_flags & 0x01: print("    - EWARN: Error Warning")
            if error_flags == 0:
                print("    (no errors)")
            return rx_count


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    try:
        ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.3)
        ser.read(ser.in_waiting)

        print(f"Connected to {port}")
        print("=" * 40)

        # Get initial status
        print("\n[1] Initial status:")
        send_command(ser, CMD_GET_STATUS)
        time.sleep(0.05)
        data = ser.read(ser.in_waiting)
        parse_status(data)

        # Start capture
        print("\n[2] Starting capture...")
        resp = send_command(ser, CMD_START_CAPTURE)
        if RSP_ACK in resp:
            print("  Capture started OK")

        # Wait and poll status
        print("\n[3] Listening for 5 seconds...")
        for i in range(5):
            time.sleep(1)
            data = send_command(ser, CMD_GET_STATUS)
            rx = parse_status(data)
            if rx and rx > 0:
                print(f"  >>> Received {rx} frames!")

        # Stop capture
        print("\n[4] Stopping capture...")
        send_command(ser, CMD_STOP_CAPTURE)

        ser.close()
        print("\nDone.")

    except serial.SerialException as e:
        print(f"Error: {e}")
        return 1

if __name__ == "__main__":
    main()
