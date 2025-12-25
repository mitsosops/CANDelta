#!/usr/bin/env python3
"""
CANDelta Low-level CAN Diagnostics
Reads raw MCP2515 registers to debug reception issues.
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


def send_cmd(ser, opcode):
    packet = bytes([STX, opcode, 0, ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print(f"Connected to {port}")

    # Start capture
    print("Starting capture...")
    send_cmd(ser, CMD_START_CAPTURE)

    print("\nMonitoring for 10 seconds - make sure playback is running!")
    print("Watching RX count changes...\n")

    last_rx = -1
    for i in range(20):
        data = send_cmd(ser, CMD_GET_STATUS)

        # Parse status
        if RSP_STATUS in data:
            idx = list(data).index(RSP_STATUS)
            if idx + 18 <= len(data):
                payload = data[idx+2:idx+18]
                rx_count = int.from_bytes(payload[6:10], 'little')
                error_flags = payload[5]

                tx_count = int.from_bytes(payload[10:14], 'little')

                if rx_count != last_rx or i == 0:
                    print(f"[{i*0.5:.1f}s] Queued: {rx_count}, Sent to host: {tx_count}, Errors: 0x{error_flags:02X}")
                    last_rx = rx_count

        time.sleep(0.5)

    send_cmd(ser, CMD_STOP_CAPTURE)
    ser.close()

    print(f"\nFinal RX count: {last_rx}")
    if last_rx <= 1:
        print("\nPossible issues:")
        print("  - CANH/CANL swapped? Try swapping the wires")
        print("  - Bitrate mismatch? Your playback might not be 500kbps")
        print("  - No common ground? USB ground should work but verify")
        print("  - Termination? CAN bus needs 120 ohm termination at each end")


if __name__ == "__main__":
    main()
