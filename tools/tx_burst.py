#!/usr/bin/env python3
"""Send a burst of CAN frames from CANDelta to test TX path"""

import sys
import time
import serial

STX = 0x02
ETX = 0x03
CMD_STOP_CAPTURE = 0x11
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
CMD_TRANSMIT_FRAME = 0x30

MODE_NORMAL = 0


def send_cmd(ser, opcode, payload=None):
    if payload is None:
        payload = []
    packet = bytes([STX, opcode, len(payload)]) + bytes(payload) + bytes([ETX])
    ser.write(packet)
    ser.flush()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    num_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    delay_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 20  # 50 Hz default

    print(f"CANDelta TX Burst Test - {port}")
    print("=" * 50)
    print(f"Will send {num_frames} frames at {1000/delay_ms:.0f} Hz")
    print("=" * 50)

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    # Stop capture (we're only TX'ing)
    send_cmd(ser, CMD_STOP_CAPTURE)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Set speed and normal mode
    print("\nConfiguring for 500 kbps...")
    speed = 500000
    speed_bytes = list(speed.to_bytes(4, 'little'))
    send_cmd(ser, CMD_SET_SPEED, speed_bytes)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    send_cmd(ser, CMD_SET_MODE, [MODE_NORMAL])
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    print(f"\nSending {num_frames} frames...")
    print("Monitor your receiving adapter now!\n")

    # Give user time to start their receiver
    time.sleep(1)

    sent = 0
    errors = 0
    start = time.time()

    for i in range(num_frames):
        # Frame: ID=0x100+i, DLC=8, data=counter
        frame_id = 0x100 + (i % 256)
        frame_data = [i & 0xFF, (i >> 8) & 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]

        # Payload: ID(4) + flags(1) + DLC(1) + data(8) = 14 bytes
        payload = list(frame_id.to_bytes(4, 'little'))
        payload.append(0x00)  # flags: standard frame
        payload.append(8)     # DLC
        payload.extend(frame_data)

        send_cmd(ser, CMD_TRANSMIT_FRAME, payload)

        # Read response
        time.sleep(0.005)
        resp = ser.read(ser.in_waiting)

        # Check for ACK (0x02 0x80 0x00 0x03)
        if b'\x02\x80\x00\x03' in resp:
            sent += 1
        else:
            errors += 1

        # Delay between frames
        time.sleep(delay_ms / 1000.0)

        # Progress
        if (i + 1) % 10 == 0:
            print(f"\r  Sent: {sent}, Errors: {errors}", end="", flush=True)

    elapsed = time.time() - start

    ser.close()

    print(f"\n\n{'=' * 50}")
    print("RESULTS")
    print("=" * 50)
    print(f"  Frames sent:    {sent}")
    print(f"  TX errors:      {errors}")
    print(f"  Time:           {elapsed:.1f}s")
    print(f"  Effective rate: {sent/elapsed:.0f} fps")
    print("=" * 50)
    print(f"\nNow check your receiving adapter - how many did it receive?")
    print(f"If it received {sent}, your CAN bus is fine.")
    print(f"If it received less, check termination and wiring.")


if __name__ == "__main__":
    main()
