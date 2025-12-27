#!/usr/bin/env python3
"""
TEC Monitor - Watch STM32's TEC while doing TRC playback.

This script:
1. Opens COM8 for SLCAN commands (no playback)
2. Periodically sends 'E' command to get TEC/REC
3. Shows correlation between TEC and TX errors

Usage: python tec_monitor.py [PORT]
       python tec_monitor.py COM8
"""

import sys
import time
import serial
import struct
import threading

def parse_extended_status(resp):
    """Parse E command response: E{TEC:2}{REC:2}{TXok:4}{TXerr:4}\\r"""
    # Find 'E' in response
    try:
        data = resp.decode('latin-1')
        if 'E' not in data:
            return None
        idx = data.index('E')
        # E + 2 hex TEC + 2 hex REC + 4 hex TXok + 4 hex TXerr = 13 chars
        if len(data) < idx + 13:
            return None
        tec = int(data[idx+1:idx+3], 16)
        rec = int(data[idx+3:idx+5], 16)
        tx_ok = int(data[idx+5:idx+9], 16)
        tx_err = int(data[idx+9:idx+13], 16)
        return {'tec': tec, 'rec': rec, 'tx_ok': tx_ok, 'tx_err': tx_err}
    except:
        return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"

    print("=" * 60)
    print("STM32 TEC/Error Monitor")
    print("=" * 60)
    print(f"Port: {port}")
    print("Press Ctrl+C to stop")
    print("=" * 60)

    try:
        ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
    except Exception as e:
        print(f"Error opening {port}: {e}")
        return

    # Drain buffer
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Initialize CAN
    ser.write(b'C\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(b'S6\r')  # 500kbps
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(b'O\r')  # Open
    time.sleep(0.1)
    ser.read(ser.in_waiting)
    print("CAN initialized at 500kbps\n")

    # Send test frames and monitor TEC
    print(f"{'Time':>8}  {'TEC':>4}  {'REC':>4}  {'TX OK':>8}  {'TX Err':>8}  {'Delta Err':>10}")
    print("-" * 60)

    prev_err = 0
    start = time.time()

    try:
        while True:
            # Send a frame
            ser.write(b't1230112233\r')
            time.sleep(0.01)
            ser.read(ser.in_waiting)

            # Get extended status
            ser.write(b'E\r')
            time.sleep(0.02)
            resp = ser.read(ser.in_waiting)

            status = parse_extended_status(resp)
            if status:
                elapsed = time.time() - start
                delta = status['tx_err'] - prev_err
                prev_err = status['tx_err']
                print(f"{elapsed:8.1f}  {status['tec']:4d}  {status['rec']:4d}  {status['tx_ok']:8d}  {status['tx_err']:8d}  {delta:+10d}")

            time.sleep(0.5)  # Poll every 500ms

    except KeyboardInterrupt:
        print("\n\nStopped.")

    ser.write(b'C\r')
    ser.close()

if __name__ == "__main__":
    main()
