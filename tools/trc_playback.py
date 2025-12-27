#!/usr/bin/env python3
"""
Play back CANHacker .trc files via SLCAN.
Respects original timing, including multiple frames at the same timestamp.
"""

import sys
import time
import serial
from collections import defaultdict


def parse_trc_file(filepath):
    """Parse CANHacker .trc file and return list of (time_ms, can_id, data) tuples"""
    frames = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('Time'):
                continue  # Skip header

            parts = line.split()
            if len(parts) < 4:
                continue

            # Time format: "07,399" = 7.399 seconds (comma as decimal)
            time_str = parts[0].replace(',', '.')
            try:
                time_sec = float(time_str)
            except ValueError:
                continue

            time_ms = int(time_sec * 1000)

            # CAN ID (decimal in file, but represents hex value)
            try:
                can_id = int(parts[1])
            except ValueError:
                continue

            # DLC
            try:
                dlc = int(parts[2])
            except ValueError:
                continue

            # Data bytes
            data = []
            for i in range(3, 3 + dlc):
                if i < len(parts):
                    try:
                        data.append(int(parts[i], 16))
                    except ValueError:
                        data.append(0)

            frames.append((time_ms, can_id, data))

    return frames


def group_by_timestamp(frames):
    """Group frames by timestamp - frames at same ms sent together"""
    groups = defaultdict(list)
    for time_ms, can_id, data in frames:
        groups[time_ms].append((can_id, data))

    # Return sorted list of (time_ms, [(can_id, data), ...])
    return sorted(groups.items())


def slcan_init(ser, bitrate=500000):
    """Initialize SLCAN adapter"""
    ser.write(b'C\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    bitrate_map = {
        10000: 'S0', 20000: 'S1', 50000: 'S2', 100000: 'S3',
        125000: 'S4', 250000: 'S5', 500000: 'S6', 800000: 'S7', 1000000: 'S8',
    }

    cmd = bitrate_map.get(bitrate, 'S6')
    ser.write(f'{cmd}\r'.encode())
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting)
    print(f"  Set bitrate: {cmd} -> {'OK' if b'\\x07' not in resp else 'ERROR'}")

    ser.write(b'O\r')
    time.sleep(0.2)
    resp = ser.read(ser.in_waiting)
    print(f"  Open channel: O -> {'OK' if b'\\x07' not in resp else 'ERROR'}")


def slcan_send(ser, can_id, data):
    """Send a CAN frame via SLCAN protocol"""
    if can_id > 0x7FF:
        cmd = f'T{can_id:08X}{len(data):01X}'
    else:
        cmd = f't{can_id:03X}{len(data):01X}'

    for byte in data:
        cmd += f'{byte:02X}'
    cmd += '\r'

    ser.write(cmd.encode())


def slcan_close(ser):
    """Close SLCAN channel"""
    ser.write(b'C\r')
    time.sleep(0.1)


def main():
    if len(sys.argv) < 3:
        print("Usage: trc_playback.py <TRC_FILE> <COM_PORT> [speed_multiplier] [loop]")
        print("Example: trc_playback.py trace.trc COM8 1.0")
        print("         trc_playback.py trace.trc COM8 1.0 loop")
        print("  speed_multiplier: 1.0 = realtime, 2.0 = 2x speed, 0 = as fast as possible")
        print("  loop: repeat trace continuously (Ctrl+C to stop)")
        return

    trc_file = sys.argv[1]
    port = sys.argv[2]
    speed_mult = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    loop_mode = len(sys.argv) > 4 and sys.argv[4].lower() == 'loop'

    print(f"TRC Playback - {trc_file}")
    print("=" * 60)

    # Parse file
    print("Parsing TRC file...")
    frames = parse_trc_file(trc_file)
    print(f"  Loaded {len(frames)} frames")

    if not frames:
        print("No frames found!")
        return

    # Group by timestamp
    grouped = group_by_timestamp(frames)
    print(f"  Grouped into {len(grouped)} time slots")

    # Find max frames per slot
    max_per_slot = max(len(g[1]) for g in grouped)
    print(f"  Max frames per millisecond: {max_per_slot}")

    # Calculate timing info
    start_time_ms = grouped[0][0]
    end_time_ms = grouped[-1][0]
    duration_sec = (end_time_ms - start_time_ms) / 1000.0

    # Count unique IDs
    unique_ids = set(f[1] for f in frames)

    print(f"  Duration: {duration_sec:.1f}s")
    print(f"  Unique CAN IDs: {len(unique_ids)}")
    print(f"  Average rate: {len(frames)/duration_sec:.0f} fps")

    if speed_mult > 0:
        print(f"  Playback speed: {speed_mult}x (ETA: {duration_sec/speed_mult:.1f}s)")
    else:
        print(f"  Playback speed: MAX (as fast as possible)")

    if loop_mode:
        print(f"  Loop mode: ON (Ctrl+C to stop)")

    print("=" * 60)

    # Open serial
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening {port}: {e}")
        return

    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print("\nInitializing adapter...")
    slcan_init(ser, 500000)

    if loop_mode:
        print(f"\nPlaying back {len(frames)} frames in loop mode...")
    else:
        print(f"\nPlaying back {len(frames)} frames in {len(grouped)} time slots...")
    print("Start your receiver now!\n")
    time.sleep(2)

    sent = 0
    errors = 0
    loop_count = 0
    playback_start = time.time()
    trace_start_ms = grouped[0][0]

    try:
        while True:
            loop_start = time.time()
            loop_count += 1

            for slot_idx, (slot_time_ms, slot_frames) in enumerate(grouped):
                # Calculate when this slot should be sent (relative to trace start)
                relative_time_ms = slot_time_ms - trace_start_ms

                if speed_mult > 0:
                    # Wait until it's time to send this slot
                    target_time = loop_start + (relative_time_ms / 1000.0) / speed_mult
                    now = time.time()
                    if target_time > now:
                        time.sleep(target_time - now)

                # Send all frames in this slot with minimal delay to prevent TX overflow
                for can_id, data in slot_frames:
                    slcan_send(ser, can_id, data)
                    sent += 1
                    # At 500kbps, an 8-byte frame takes ~0.23ms on wire
                    # Give STM32 time to queue frame before next one
                    if len(slot_frames) > 1:
                        time.sleep(0.0003)  # 0.3ms between frames in burst

                # After sending the burst, flush and check for errors
                ser.flush()

                # Progress every 500 slots
                if (slot_idx + 1) % 500 == 0:
                    # Check for errors
                    if ser.in_waiting > 0:
                        resp = ser.read(ser.in_waiting)
                        errors += resp.count(b'\x07')

                    elapsed = time.time() - playback_start
                    if loop_mode:
                        print(f"\r  Loop {loop_count} - Sent: {sent}, Errors: {errors}, Elapsed: {elapsed:.1f}s", end="", flush=True)
                    else:
                        pct = 100 * (slot_idx + 1) / len(grouped)
                        print(f"\r  {pct:.0f}% - Sent: {sent}, Errors: {errors}, Elapsed: {elapsed:.1f}s", end="", flush=True)

            # End of one loop iteration
            if not loop_mode:
                break

            # Small gap between loops
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    elapsed = time.time() - playback_start

    # Drain any remaining responses
    time.sleep(0.2)
    resp = ser.read(ser.in_waiting)
    if resp:
        errors += resp.count(b'\x07')

    slcan_close(ser)
    ser.close()

    print(f"\n{'=' * 60}")
    print("PLAYBACK COMPLETE")
    print("=" * 60)
    print(f"  Loops:          {loop_count}")
    print(f"  Frames sent:    {sent}")
    print(f"  TX errors:      {errors}")
    print(f"  Elapsed time:   {elapsed:.1f}s")
    print(f"  Effective rate: {sent/elapsed:.0f} fps")
    print("=" * 60)


if __name__ == "__main__":
    main()
