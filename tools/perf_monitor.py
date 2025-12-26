#!/usr/bin/env python3
"""Real-time performance monitor for CANDelta"""

import sys
import time
import serial
import struct

STX = 0x02
ETX = 0x03
CMD_GET_PERF_STATS = 0x07
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
RSP_PERF_STATS = 0x86


def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)


def parse_packet(data, expected_opcode):
    """Parse a properly framed STX/ETX packet"""
    # Find STX followed by expected opcode
    for i in range(len(data) - 3):
        if data[i] == STX and data[i + 1] == expected_opcode:
            length = data[i + 2]
            packet_end = i + 3 + length
            if packet_end < len(data) and data[packet_end] == ETX:
                # Valid packet found
                return data[i + 3 : i + 3 + length]
    return None


def get_perf_stats(ser):
    data = send_cmd(ser, CMD_GET_PERF_STATS)

    payload = parse_packet(data, RSP_PERF_STATS)
    if payload and len(payload) >= 13:
        fps = struct.unpack("<I", payload[0:4])[0]
        peak_fps = struct.unpack("<I", payload[4:8])[0]
        dropped = struct.unpack("<I", payload[8:12])[0]
        buffer_util = payload[12]
        return {
            "fps": fps,
            "peak_fps": peak_fps,
            "dropped": dropped,
            "buffer_util": buffer_util,
        }
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print(f"CANDelta Performance Monitor - {port}")
    print("=" * 60)
    print("Start your CAN playback to see performance metrics")
    print("=" * 60)

    # Start capture
    send_cmd(ser, CMD_START_CAPTURE)

    print(f"\n{'Time':>6} | {'FPS':>6} | {'Peak':>6} | {'Dropped':>8} | {'Buffer':>7} |")
    print("-" * 60)

    start_time = time.time()
    last_dropped = 0
    alerts = []

    try:
        while time.time() - start_time < duration:
            stats = get_perf_stats(ser)
            if stats:
                elapsed = time.time() - start_time

                # Check for alerts
                alert_str = ""
                if stats["buffer_util"] > 80:
                    alert_str += " BUFFER HIGH!"
                if stats["dropped"] > last_dropped:
                    alert_str += " FRAMES DROPPED!"
                    last_dropped = stats["dropped"]

                # Create buffer bar
                bar_len = stats["buffer_util"] // 5  # 20 chars max
                bar = "#" * bar_len + "-" * (20 - bar_len)

                print(
                    f"{elapsed:5.1f}s | {stats['fps']:>6} | {stats['peak_fps']:>6} | "
                    f"{stats['dropped']:>8} | [{bar}] {stats['buffer_util']:>3}%{alert_str}"
                )

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    # Final stats
    stats = get_perf_stats(ser)
    send_cmd(ser, CMD_STOP_CAPTURE)

    print("\n" + "=" * 60)
    print("FINAL SUMMARY")
    print("=" * 60)
    if stats:
        print(f"  Peak FPS:        {stats['peak_fps']}")
        print(f"  Dropped Frames:  {stats['dropped']}")

        # Evaluation
        print("\nEvaluation:")
        if stats["peak_fps"] >= 200:
            print("  [PASS] Peak FPS >= 200")
        else:
            print(f"  [WARN] Peak FPS = {stats['peak_fps']} (target: 200+)")

        if stats["dropped"] == 0:
            print("  [PASS] No frames dropped")
        else:
            print(f"  [FAIL] {stats['dropped']} frames dropped!")

    ser.close()
    print("\nDone!")


if __name__ == "__main__":
    main()
