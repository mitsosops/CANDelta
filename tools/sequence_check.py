#!/usr/bin/env python3
"""
Capture frames and check for missing sequence numbers.
Have your adapter send frames with sequential counter in first byte of data.
"""

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

    print(f"Sequence Check - {port}")
    print("=" * 50)
    print("Send frames with sequential counter in data[0]")
    print("=" * 50)

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    # Setup
    ser.write(bytes([STX, CMD_STOP_CAPTURE, 0, ETX]))
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    speed = 500000
    sb = speed.to_bytes(4, 'little')
    ser.write(bytes([STX, CMD_SET_SPEED, 4]) + sb + bytes([ETX]))
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(bytes([STX, CMD_START_CAPTURE, 0, ETX]))
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    print(f"\nCapturing for {duration} seconds...")
    print("Send your sequential frames now!\n")

    frames = []  # List of (id, data[0]) tuples

    # Parser state
    state = 0
    opcode = 0
    length = 0
    payload_remaining = 0
    payload = []

    start = time.time()
    try:
        while time.time() - start < duration:
            data = ser.read(1024)
            if data:
                for b in data:
                    if state == 0:  # Wait STX
                        if b == STX:
                            state = 1
                    elif state == 1:  # Opcode
                        opcode = b
                        state = 2
                    elif state == 2:  # Length
                        length = b
                        payload_remaining = length
                        payload = []
                        if payload_remaining == 0:
                            state = 4
                        else:
                            state = 3
                    elif state == 3:  # Payload
                        payload.append(b)
                        payload_remaining -= 1
                        if payload_remaining <= 0:
                            state = 4
                    elif state == 4:  # ETX
                        if b == ETX and opcode == RSP_CAN_FRAME and len(payload) >= 15:
                            # Parse frame: timestamp(8) + id(4) + flags(1) + dlc(1) + data
                            frame_id = payload[8] | (payload[9] << 8) | (payload[10] << 16) | (payload[11] << 24)
                            dlc = payload[13]
                            if dlc > 0:
                                seq = payload[14]  # data[0] = sequence number
                                frames.append((frame_id, seq))
                        state = 0

            elapsed = time.time() - start
            if int(elapsed) != int(elapsed - 0.1):
                print(f"\r  {elapsed:.0f}s: {len(frames)} frames", end="", flush=True)

    except KeyboardInterrupt:
        print("\nInterrupted")

    ser.write(bytes([STX, CMD_STOP_CAPTURE, 0, ETX]))
    ser.close()

    print(f"\n\n{'=' * 50}")
    print("ANALYSIS")
    print("=" * 50)

    if not frames:
        print("No frames received!")
        return

    print(f"Total frames received: {len(frames)}")

    # Extract sequences and find gaps
    sequences = [f[1] for f in frames]

    if len(sequences) < 2:
        print("Not enough frames to analyze")
        return

    # Find min and max to determine expected range
    min_seq = min(sequences)
    max_seq = max(sequences)

    print(f"Sequence range: {min_seq} to {max_seq}")

    # Check for missing sequences (assuming 8-bit counter)
    expected = set(range(min_seq, max_seq + 1))
    received = set(sequences)
    missing = sorted(expected - received)
    duplicates = [s for s in sequences if sequences.count(s) > 1]

    if missing:
        print(f"\nMISSING SEQUENCES ({len(missing)}):")
        # Group consecutive missing
        groups = []
        start_group = missing[0]
        end_group = missing[0]
        for m in missing[1:]:
            if m == end_group + 1:
                end_group = m
            else:
                groups.append((start_group, end_group))
                start_group = m
                end_group = m
        groups.append((start_group, end_group))

        for g in groups:
            if g[0] == g[1]:
                print(f"  {g[0]}")
            else:
                print(f"  {g[0]}-{g[1]} ({g[1]-g[0]+1} frames)")
    else:
        print("\nNo missing sequences - all frames received!")

    if duplicates:
        print(f"\nDuplicate sequences: {set(duplicates)}")

    # Analyze pattern
    print(f"\n{'=' * 50}")
    expected_count = max_seq - min_seq + 1
    received_count = len(frames)
    loss_pct = 100 * (expected_count - received_count) / expected_count if expected_count > 0 else 0

    print(f"Expected: {expected_count} frames")
    print(f"Received: {received_count} frames")
    print(f"Lost:     {expected_count - received_count} frames ({loss_pct:.1f}%)")

    if missing:
        # Check if losses are at start
        at_start = sum(1 for m in missing if m < min_seq + 10)
        at_end = sum(1 for m in missing if m > max_seq - 10)
        in_middle = len(missing) - at_start - at_end

        print(f"\nLoss pattern:")
        print(f"  At start (first 10): {at_start}")
        print(f"  In middle:           {in_middle}")
        print(f"  At end (last 10):    {at_end}")


if __name__ == "__main__":
    main()
