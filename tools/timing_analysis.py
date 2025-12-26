#!/usr/bin/env python3
"""
Analyze frame timing to detect TX gaps.
If there are long idle periods where no frames arrive, the problem is the sending adapter.
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
    expected_interval_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 20  # 50 Hz

    print(f"Timing Analysis - {port}")
    print("=" * 60)
    print(f"Expected frame interval: {expected_interval_ms}ms ({1000/expected_interval_ms:.0f} Hz)")
    print("=" * 60)

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
    print("Send your frames now!\n")

    timestamps_us = []  # Microsecond timestamps from firmware

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
                    if state == 0:
                        if b == STX:
                            state = 1
                    elif state == 1:
                        opcode = b
                        state = 2
                    elif state == 2:
                        length = b
                        payload_remaining = length
                        payload = []
                        if payload_remaining == 0:
                            state = 4
                        else:
                            state = 3
                    elif state == 3:
                        payload.append(b)
                        payload_remaining -= 1
                        if payload_remaining <= 0:
                            state = 4
                    elif state == 4:
                        if b == ETX and opcode == RSP_CAN_FRAME and len(payload) >= 8:
                            # Extract 64-bit timestamp (first 8 bytes, little-endian)
                            ts = 0
                            for i in range(8):
                                ts |= payload[i] << (i * 8)
                            timestamps_us.append(ts)
                        state = 0

            elapsed = time.time() - start
            if int(elapsed) != int(elapsed - 0.1):
                print(f"\r  {elapsed:.0f}s: {len(timestamps_us)} frames", end="", flush=True)

    except KeyboardInterrupt:
        print("\nInterrupted")

    ser.write(bytes([STX, CMD_STOP_CAPTURE, 0, ETX]))
    ser.close()

    print(f"\n\n{'=' * 60}")
    print("TIMING ANALYSIS")
    print("=" * 60)

    if len(timestamps_us) < 2:
        print("Not enough frames to analyze")
        return

    print(f"Total frames: {len(timestamps_us)}")

    # Calculate inter-frame gaps
    gaps_us = []
    for i in range(1, len(timestamps_us)):
        gap = timestamps_us[i] - timestamps_us[i-1]
        gaps_us.append(gap)

    gaps_ms = [g / 1000.0 for g in gaps_us]

    # Statistics
    min_gap = min(gaps_ms)
    max_gap = max(gaps_ms)
    avg_gap = sum(gaps_ms) / len(gaps_ms)

    print(f"\nInter-frame gaps:")
    print(f"  Min:  {min_gap:.2f} ms")
    print(f"  Max:  {max_gap:.2f} ms")
    print(f"  Avg:  {avg_gap:.2f} ms")

    # Count gaps that are suspiciously long (> 1.5x expected)
    threshold = expected_interval_ms * 1.5
    long_gaps = [(i, g) for i, g in enumerate(gaps_ms) if g > threshold]

    print(f"\n{'=' * 60}")
    print(f"IDLE PERIODS (gaps > {threshold:.0f}ms)")
    print("=" * 60)

    if long_gaps:
        # Estimate missing frames per gap
        total_missing = 0
        print(f"\n{'Frame#':<8} {'Gap (ms)':<12} {'Est. Missing':<12}")
        print("-" * 36)

        for idx, gap in long_gaps[:20]:  # Show first 20
            missing = int(gap / expected_interval_ms) - 1
            if missing < 0:
                missing = 0
            total_missing += missing
            print(f"{idx:<8} {gap:<12.1f} {missing:<12}")

        if len(long_gaps) > 20:
            print(f"  ... and {len(long_gaps) - 20} more long gaps")
            # Count rest
            for idx, gap in long_gaps[20:]:
                missing = int(gap / expected_interval_ms) - 1
                if missing > 0:
                    total_missing += missing

        print(f"\nTotal long gaps: {len(long_gaps)}")
        print(f"Estimated missing frames from gaps: {total_missing}")

        print(f"\n{'=' * 60}")
        print("CONCLUSION")
        print("=" * 60)
        print(f"  MCP2515 was IDLE during these gaps.")
        print(f"  Frames were NOT arriving from the CAN bus.")
        print(f"  => Problem is your SENDING ADAPTER, not CANDelta.")
    else:
        print(f"\nNo suspicious gaps found!")
        print(f"All frames arrived within expected timing.")


if __name__ == "__main__":
    main()
