#!/usr/bin/env python3
"""
CANDelta CAN Capture Test
Listens for CAN frames and prints them as they arrive.

Usage: python capture_test.py COM7
Press Ctrl+C to stop.
"""

import sys
import time
import serial

# Protocol constants
STX = 0x02
ETX = 0x03

CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11

RSP_ACK = 0x80
RSP_CAN_FRAME = 0x84


def send_command(ser, opcode, params=b''):
    """Send a command."""
    packet = bytes([STX, opcode, len(params)]) + params + bytes([ETX])
    ser.write(packet)
    ser.flush()


def main():
    if len(sys.argv) < 2:
        print("Usage: python capture_test.py <COM_PORT>")
        return 1

    port = sys.argv[1]
    print(f"Connecting to {port}...")

    try:
        ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.3)

        # Clear any pending data
        ser.read(ser.in_waiting)

        print("Starting capture...")
        send_command(ser, CMD_START_CAPTURE)
        time.sleep(0.1)

        # Check for ACK
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            if RSP_ACK in data:
                print("Capture started - waiting for CAN frames...")
            else:
                print(f"Unexpected response: {data.hex()}")

        print("-" * 60)
        print("Timestamp (us)      ID         DLC  Data")
        print("-" * 60)

        frame_count = 0
        buffer = b''

        try:
            while True:
                if ser.in_waiting:
                    buffer += ser.read(ser.in_waiting)

                # Process complete packets
                while STX in buffer and ETX in buffer:
                    stx_idx = buffer.index(STX)
                    etx_idx = buffer.index(ETX, stx_idx)

                    packet = buffer[stx_idx+1:etx_idx]
                    buffer = buffer[etx_idx+1:]

                    if len(packet) >= 2:
                        opcode = packet[0]
                        length = packet[1]
                        payload = packet[2:2+length]

                        if opcode == RSP_CAN_FRAME and len(payload) >= 14:
                            # Parse CAN frame
                            timestamp = int.from_bytes(payload[0:8], 'little')
                            can_id = int.from_bytes(payload[8:12], 'little')
                            flags = payload[12]
                            dlc = payload[13]
                            data = payload[14:14+dlc]

                            extended = bool(flags & 0x01)
                            rtr = bool(flags & 0x02)

                            id_str = f"0x{can_id:08X}" if extended else f"0x{can_id:03X}"
                            data_str = ' '.join(f'{b:02X}' for b in data)
                            rtr_str = " RTR" if rtr else ""

                            print(f"{timestamp:>16}  {id_str:>10}  {dlc:>3}  {data_str}{rtr_str}")
                            frame_count += 1

                time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n" + "-" * 60)
            print(f"Received {frame_count} frames")

        print("Stopping capture...")
        send_command(ser, CMD_STOP_CAPTURE)
        time.sleep(0.1)
        ser.close()
        print("Done.")

    except serial.SerialException as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
