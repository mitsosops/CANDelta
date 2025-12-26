#!/usr/bin/env python3
"""Raw frame counter - counts CAN frame packets with minimal parsing"""

import sys
import time
import serial

STX = 0x02
ETX = 0x03
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_SET_SPEED = 0x20
RSP_CAN_FRAME = 0x84


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30

    print(f"Raw Frame Counter - {port}")
    print("=" * 40)

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    # Stop, set speed, start
    ser.write(bytes([STX, CMD_STOP_CAPTURE, 0, ETX]))
    ser.flush()
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Set 500kbps
    speed = 500000
    speed_bytes = speed.to_bytes(4, 'little')
    ser.write(bytes([STX, CMD_SET_SPEED, 4]) + speed_bytes + bytes([ETX]))
    ser.flush()
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Start capture
    ser.write(bytes([STX, CMD_START_CAPTURE, 0, ETX]))
    ser.flush()
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    print(f"Capturing for {duration} seconds...")
    print("Send your burst now!\n")

    frame_count = 0
    bytes_received = 0
    start = time.time()

    # Simple state machine to count frame packets
    state = 0  # 0=wait STX, 1=read opcode, 2=read len, 3=skip payload, 4=wait ETX
    opcode = 0
    length = 0
    payload_remaining = 0

    try:
        while time.time() - start < duration:
            data = ser.read(1024)
            if data:
                bytes_received += len(data)
                for b in data:
                    if state == 0:  # Wait for STX
                        if b == STX:
                            state = 1
                    elif state == 1:  # Read opcode
                        opcode = b
                        state = 2
                    elif state == 2:  # Read length
                        length = b
                        payload_remaining = length
                        if payload_remaining == 0:
                            state = 4  # Skip payload, go to ETX
                        else:
                            state = 3
                    elif state == 3:  # Skip payload
                        payload_remaining -= 1
                        if payload_remaining <= 0:
                            state = 4
                    elif state == 4:  # Wait for ETX
                        if b == ETX:
                            if opcode == RSP_CAN_FRAME:
                                frame_count += 1
                        state = 0

            # Print progress every second
            elapsed = time.time() - start
            if int(elapsed) > int(elapsed - 0.1):
                print(f"\r  {elapsed:.1f}s: {frame_count} frames, {bytes_received} bytes", end="", flush=True)

    except KeyboardInterrupt:
        print("\nInterrupted")

    # Stop capture
    ser.write(bytes([STX, CMD_STOP_CAPTURE, 0, ETX]))
    ser.flush()
    ser.close()

    print(f"\n\n{'=' * 40}")
    print(f"RESULT: {frame_count} CAN frames received")
    print(f"        {bytes_received} total bytes")
    print("=" * 40)


if __name__ == "__main__":
    main()
