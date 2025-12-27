#!/usr/bin/env python3
"""
SLCAN/LAWICEL protocol TX tool for CANHacker-compatible adapters.
Works with MK20DX256-based adapters running CANHacker firmware.
"""

import sys
import time
import serial


def slcan_init(ser, bitrate=500000):
    """Initialize SLCAN adapter"""
    # Close any open channel first (ignore errors)
    ser.write(b'C\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Set bitrate
    # S0=10k, S1=20k, S2=50k, S3=100k, S4=125k, S5=250k, S6=500k, S7=800k, S8=1M
    bitrate_map = {
        10000: 'S0',
        20000: 'S1',
        50000: 'S2',
        100000: 'S3',
        125000: 'S4',
        250000: 'S5',
        500000: 'S6',
        800000: 'S7',
        1000000: 'S8',
    }

    cmd = bitrate_map.get(bitrate, 'S6')  # Default 500k
    ser.write(f'{cmd}\r'.encode())
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting)
    ok = b'\x07' not in resp
    print(f"  Set bitrate: {cmd} -> {'OK' if ok else 'ERROR'}")

    # Open channel
    ser.write(b'O\r')
    time.sleep(0.2)
    resp = ser.read(ser.in_waiting)
    ok = b'\x07' not in resp
    print(f"  Open channel: O -> {'OK' if ok else 'ERROR'}")


def slcan_send(ser, can_id, data, extended=False):
    """Send a CAN frame via SLCAN protocol"""
    if extended:
        # Extended frame: Tiiiiiiiildd...\r
        cmd = f'T{can_id:08X}{len(data):01X}'
    else:
        # Standard frame: tiiildd...\r
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
    if len(sys.argv) < 2:
        print("Usage: slcan_tx.py <COM_PORT> [num_frames] [interval_ms]")
        print("       slcan_tx.py <COM_PORT> loop [interval_ms]  - continuous mode")
        print("Example: slcan_tx.py COM3 100 20")
        print("         slcan_tx.py COM8 loop 20")
        return

    port = sys.argv[1]
    loop_mode = len(sys.argv) > 2 and sys.argv[2].lower() == 'loop'

    if loop_mode:
        num_frames = 0  # infinite
        interval_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 20
    else:
        num_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 100
        interval_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 20

    print(f"SLCAN TX Test - {port}")
    print("=" * 50)
    if loop_mode:
        print(f"Mode: CONTINUOUS (Ctrl+C to stop), Interval: {interval_ms}ms ({1000/interval_ms:.0f} Hz)")
    else:
        print(f"Frames: {num_frames}, Interval: {interval_ms}ms ({1000/interval_ms:.0f} Hz)")
    print("=" * 50)

    # Open serial - CANHacker typically uses 115200 or 1000000 baud
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except:
        try:
            ser = serial.Serial(port, 1000000, timeout=0.5)
            print("  Using 1M baud")
        except Exception as e:
            print(f"Error opening {port}: {e}")
            return

    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print("\nInitializing adapter...")
    slcan_init(ser, 500000)

    if loop_mode:
        print(f"\nSending continuous frames with ID=0x123...")
    else:
        print(f"\nSending {num_frames} frames with ID=0x123...")
    print("Monitor on CANDelta now!\n")
    time.sleep(1)

    sent = 0
    errors = 0
    start = time.time()
    i = 0

    try:
        while loop_mode or i < num_frames:
            # Frame: ID=0x123, data=sequence counter
            data = [i & 0xFF, (i >> 8) & 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]

            slcan_send(ser, 0x123, data)
            sent += 1

            # Small delay to let serial TX complete
            time.sleep(0.002)

            # Check for response/error - \r means OK, \x07 means error
            resp = ser.read(ser.in_waiting)
            if resp and b'\x07' in resp:  # BEL = error
                errors += 1

            # Inter-frame delay
            time.sleep(interval_ms / 1000.0)

            if (i + 1) % 10 == 0:
                elapsed = time.time() - start
                rate = sent / elapsed if elapsed > 0 else 0
                print(f"\r  Sent: {sent}, Errors: {errors}, Rate: {rate:.0f} fps", end="", flush=True)

            i += 1

    except KeyboardInterrupt:
        print("\n\nStopped by user.")

    elapsed = time.time() - start

    slcan_close(ser)
    ser.close()

    print(f"\n{'=' * 50}")
    print("RESULTS")
    print("=" * 50)
    print(f"  Sent: {sent}")
    print(f"  Errors: {errors}")
    print(f"  Time: {elapsed:.1f}s")
    print(f"  Rate: {sent/elapsed:.0f} fps")
    print("=" * 50)


if __name__ == "__main__":
    main()
