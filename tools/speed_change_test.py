#!/usr/bin/env python3
"""Test changing speed on the fly and continuing capture"""

import sys
import time
import serial
import struct

STX = 0x02
ETX = 0x03
CMD_SET_SPEED = 0x20
CMD_DEBUG = 0x06
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
RSP_ACK = 0x80
RSP_DEBUG = 0x85


def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.1)
    return ser.read(ser.in_waiting)


def set_speed(ser, speed_bps):
    params = list(struct.pack("<I", speed_bps))
    data = send_cmd(ser, CMD_SET_SPEED, params)
    return RSP_ACK in data


def get_debug(ser):
    data = send_cmd(ser, CMD_DEBUG)
    if RSP_DEBUG in data:
        idx = list(data).index(RSP_DEBUG)
        if idx + 14 <= len(data):
            payload = data[idx + 2 : idx + 14]
            return {
                "rx_count": payload[4],
                "tx_count": payload[5],
                "canintf": payload[7],
                "eflg": payload[9],
                "cnf1": payload[10],
            }
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print(f"Connected to {port}")
    print("=" * 60)

    # Start at 500kbps
    print("\n1. Setting speed to 500 kbps...")
    if set_speed(ser, 500000):
        print("   OK")
    else:
        print("   FAILED")
        return

    # Start capture
    print("2. Starting capture...")
    send_cmd(ser, CMD_START_CAPTURE)

    # Monitor for 3 seconds
    print("3. Capturing at 500 kbps for 3 seconds...")
    print("   (Make sure playback is running at 500 kbps)")
    total_500 = 0
    for i in range(6):
        debug = get_debug(ser)
        if debug:
            total_500 += debug["tx_count"]
            status = "MERRF " if debug["canintf"] & 0x80 else ""
            print(f"   [{i*0.5:.1f}s] sent={debug['tx_count']:3d} CNF1=0x{debug['cnf1']:02X} {status}")
        time.sleep(0.5)

    print(f"   Total at 500 kbps: {total_500} frames")

    # Change speed to 250kbps on the fly
    print("\n" + "!" * 60)
    print("!!! CHANGE YOUR ADAPTER TO 250 kbps NOW !!!")
    print("!" * 60)
    for i in range(10, 0, -1):
        print(f"   Changing in {i}...", end="\r")
        time.sleep(1)
    print("   Changing CANDelta to 250 kbps...     ")
    if set_speed(ser, 250000):
        print("   OK - Speed changed")
    else:
        print("   FAILED to change speed")
        return

    # Continue capturing at new speed
    print("5. Capturing at 250 kbps for 3 seconds...")
    total_250 = 0
    for i in range(6):
        debug = get_debug(ser)
        if debug:
            total_250 += debug["tx_count"]
            status = "MERRF " if debug["canintf"] & 0x80 else ""
            print(f"   [{i*0.5:.1f}s] sent={debug['tx_count']:3d} CNF1=0x{debug['cnf1']:02X} {status}")
        time.sleep(0.5)

    print(f"   Total at 250 kbps: {total_250} frames")

    # Change back to 500kbps
    print("\n" + "!" * 60)
    print("!!! CHANGE YOUR ADAPTER BACK TO 500 kbps NOW !!!")
    print("!" * 60)
    for i in range(10, 0, -1):
        print(f"   Changing in {i}...", end="\r")
        time.sleep(1)
    print("   Changing CANDelta to 500 kbps...     ")
    if set_speed(ser, 500000):
        print("   OK - Speed changed")
    else:
        print("   FAILED")
        return

    # Final capture
    print("7. Capturing at 500 kbps for 3 seconds...")
    total_final = 0
    for i in range(6):
        debug = get_debug(ser)
        if debug:
            total_final += debug["tx_count"]
            status = "MERRF " if debug["canintf"] & 0x80 else ""
            print(f"   [{i*0.5:.1f}s] sent={debug['tx_count']:3d} CNF1=0x{debug['cnf1']:02X} {status}")
        time.sleep(0.5)

    print(f"   Total at 500 kbps: {total_final} frames")

    # Stop capture
    send_cmd(ser, CMD_STOP_CAPTURE)

    print("\n" + "=" * 60)
    print("SUMMARY:")
    print(f"  500 kbps (first):  {total_500} frames")
    print(f"  250 kbps (middle): {total_250} frames")
    print(f"  500 kbps (final):  {total_final} frames")

    if total_500 > 0 and total_250 > 0 and total_final > 0:
        print("\nSUCCESS: Speed changes on the fly working!")
    else:
        print("\nWARNING: Some captures had no frames")

    ser.close()


if __name__ == "__main__":
    main()
