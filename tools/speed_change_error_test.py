#!/usr/bin/env python3
"""
Speed Change Error Test - Watch STM32 errors during CANDelta speed changes.

This test:
1. Starts TRC playback on STM32 (COM8)
2. Changes CANDelta speed (COM7) to cause/stop errors
3. Shows error rate correlation with speed changes
4. Monitors STM32 TEC to see if it recovers

Usage: python speed_change_error_test.py [DELTA_PORT] [STM32_PORT]
       python speed_change_error_test.py COM7 COM8
"""

import sys
import time
import serial
import struct
import threading

# CANDelta protocol
STX = 0x02
ETX = 0x03
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
RSP_ACK = 0x80
MODE_NORMAL = 0x00

def send_cmd(ser, opcode, params=None):
    """Send command to CANDelta"""
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)

def set_delta_speed(ser, speed):
    """Set CANDelta to specified speed"""
    params = list(struct.pack("<I", speed))
    resp = send_cmd(ser, CMD_SET_SPEED, params)
    return RSP_ACK in resp

def parse_extended_status(resp):
    """Parse E command response: E{TEC:2}{REC:2}{TXok:4}{TXerr:4}\\r"""
    try:
        data = resp.decode('latin-1')
        if 'E' not in data:
            return None
        idx = data.index('E')
        if len(data) < idx + 13:
            return None
        tec = int(data[idx+1:idx+3], 16)
        rec = int(data[idx+3:idx+5], 16)
        tx_ok = int(data[idx+5:idx+9], 16)
        tx_err = int(data[idx+9:idx+13], 16)
        return {'tec': tec, 'rec': rec, 'tx_ok': tx_ok, 'tx_err': tx_err}
    except:
        return None

# Shared state for playback thread
playback_stats = {'sent': 0, 'errors': 0, 'running': True, 'paused': False, 'tec': 0}
stats_lock = threading.Lock()
stm32_ser = None  # Shared serial for TEC queries

def playback_thread(port, trc_file):
    """Run TRC playback in background and track errors"""
    global playback_stats, stm32_ser

    try:
        stm32_ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
    except Exception as e:
        print(f"Playback thread: Error opening {port}: {e}")
        return

    # Init CAN
    stm32_ser.write(b'C\r')
    time.sleep(0.1)
    stm32_ser.read(stm32_ser.in_waiting)
    stm32_ser.write(b'S6\r')  # 500kbps
    time.sleep(0.1)
    stm32_ser.read(stm32_ser.in_waiting)
    stm32_ser.write(b'O\r')
    time.sleep(0.1)
    stm32_ser.read(stm32_ser.in_waiting)

    # Parse TRC file (simplified)
    frames = []
    with open(trc_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(';'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    msg_id = int(parts[2], 16)
                    dlc = int(parts[3])
                    data = [int(b, 16) for b in parts[4:4+dlc]]
                    frames.append((msg_id, dlc, data))
                except:
                    continue

    print(f"  Loaded {len(frames)} frames from TRC")

    # Playback loop
    idx = 0
    tec_check_counter = 0
    while playback_stats['running']:
        if playback_stats['paused']:
            time.sleep(0.01)
            continue

        frame = frames[idx % len(frames)]
        msg_id, dlc, data = frame

        # Build SLCAN command
        cmd = f"t{msg_id:03X}{dlc}" + "".join(f"{b:02X}" for b in data) + "\r"
        stm32_ser.write(cmd.encode())

        # Check response
        time.sleep(0.001)
        resp = stm32_ser.read(stm32_ser.in_waiting)

        with stats_lock:
            playback_stats['sent'] += 1
            if b'\x07' in resp:
                playback_stats['errors'] += resp.count(b'\x07')

        idx += 1

        # Periodically check TEC (every 100 frames)
        tec_check_counter += 1
        if tec_check_counter >= 100:
            tec_check_counter = 0
            stm32_ser.write(b'E\r')
            time.sleep(0.01)
            resp = stm32_ser.read(stm32_ser.in_waiting)
            status = parse_extended_status(resp)
            if status:
                with stats_lock:
                    playback_stats['tec'] = status['tec']

        if idx % 100 == 0:
            time.sleep(0.01)  # Brief pause every 100 frames

    stm32_ser.write(b'C\r')
    stm32_ser.close()

def main():
    delta_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    stm32_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"
    trc_file = r"C:\Users\Mitsos\Desktop\maxxecu_opel2.canhacker.trc"

    print("=" * 80)
    print("SPEED CHANGE ERROR CORRELATION TEST (with TEC monitoring)")
    print("=" * 80)
    print(f"CANDelta: {delta_port}")
    print(f"STM32:    {stm32_port}")
    print("=" * 80)

    # Open CANDelta
    try:
        delta = serial.Serial(port=delta_port, baudrate=115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening CANDelta: {e}")
        return

    delta.dtr = True
    delta.rts = True
    time.sleep(0.3)
    delta.read(delta.in_waiting)

    # Set CANDelta to 500k normal mode
    print("\nInitializing CANDelta at 500kbps...")
    set_delta_speed(delta, 500000)
    send_cmd(delta, CMD_SET_MODE, [MODE_NORMAL])

    # Start playback thread
    print("Starting STM32 playback thread...")
    playback_stats['running'] = True
    playback_stats['sent'] = 0
    playback_stats['errors'] = 0
    playback_stats['tec'] = 0

    thread = threading.Thread(target=playback_thread, args=(stm32_port, trc_file))
    thread.start()
    time.sleep(2)  # Let playback stabilize

    print("\n" + "-" * 80)
    print("PHASE 1: CANDelta at 500k (should ACK) - watching for 5 seconds")
    print("-" * 80)
    print(f"{'Time':>6}  {'Sent':>8}  {'Errors':>8}  {'TEC':>4}  {'Err Rate':>10}")

    for i in range(10):
        time.sleep(0.5)
        with stats_lock:
            print(f"{i*0.5:6.1f}  {playback_stats['sent']:8d}  {playback_stats['errors']:8d}  {playback_stats['tec']:4d}")

    baseline_errors = playback_stats['errors']
    print(f"\nBaseline errors after 5s: {baseline_errors}")

    print("\n" + "-" * 80)
    print("PHASE 2: CANDelta switching to 250k (no ACKs) - watching for 5 seconds")
    print("-" * 80)
    print(f"{'Time':>6}  {'Sent':>8}  {'Errors':>8}  {'TEC':>4}  {'Err Rate':>10}")

    if set_delta_speed(delta, 250000):
        print("  >>> CANDelta now at 250kbps (wrong speed)")

    prev_errors = playback_stats['errors']
    for i in range(10):
        time.sleep(0.5)
        with stats_lock:
            curr_errors = playback_stats['errors']
            delta_err = curr_errors - prev_errors
            prev_errors = curr_errors
            print(f"{5+i*0.5:6.1f}  {playback_stats['sent']:8d}  {curr_errors:8d}  {playback_stats['tec']:4d}  +{delta_err}")

    errors_after_wrong_speed = playback_stats['errors']
    print(f"\nErrors after wrong speed: {errors_after_wrong_speed} (+{errors_after_wrong_speed - baseline_errors})")

    print("\n" + "-" * 80)
    print("PHASE 3: CANDelta back to 500k (ACKs resume) - watching for 15 seconds")
    print("-" * 80)
    print(f"{'Time':>6}  {'Sent':>8}  {'Errors':>8}  {'TEC':>4}  {'Err Rate':>10}")

    if set_delta_speed(delta, 500000):
        print("  >>> CANDelta now at 500kbps (correct speed)")

    prev_errors = playback_stats['errors']
    stable_count = 0
    recovery_time = None

    for i in range(30):
        time.sleep(0.5)
        with stats_lock:
            curr_errors = playback_stats['errors']
            delta_err = curr_errors - prev_errors
            tec = playback_stats['tec']
            status = "STABLE" if delta_err == 0 else f"+{delta_err}"
            print(f"{10+i*0.5:6.1f}  {playback_stats['sent']:8d}  {curr_errors:8d}  {tec:4d}  {status}")

            if delta_err == 0:
                stable_count += 1
                if recovery_time is None and stable_count >= 3:
                    recovery_time = 10 + i * 0.5
            else:
                stable_count = 0
            prev_errors = curr_errors

    # Stop playback
    playback_stats['running'] = False
    thread.join(timeout=2)

    print("\n" + "=" * 80)
    print("RESULTS")
    print("=" * 80)
    print(f"Total sent:   {playback_stats['sent']}")
    print(f"Total errors: {playback_stats['errors']}")
    print(f"Final TEC:    {playback_stats['tec']}")
    print(f"")
    print(f"Errors during correct speed (phase 1): {baseline_errors}")
    print(f"Errors during wrong speed (phase 2):   {errors_after_wrong_speed - baseline_errors}")
    print(f"Errors after recovery (phase 3):       {playback_stats['errors'] - errors_after_wrong_speed}")

    if recovery_time:
        print(f"\n*** Recovery detected at {recovery_time:.1f}s ***")
    elif playback_stats['errors'] - errors_after_wrong_speed == 0:
        print("\n*** PASS: Errors stopped immediately after speed corrected ***")
    else:
        print("\n*** ISSUE: Errors continued without recovery ***")

    delta.close()
    print("=" * 80)

if __name__ == "__main__":
    main()
