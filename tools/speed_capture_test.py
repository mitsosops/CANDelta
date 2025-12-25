#!/usr/bin/env python3
"""Test CAN capture at different speeds"""

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

SPEEDS = {
    "125": 125000,
    "250": 250000,
    "500": 500000,
    "1000": 1000000,
}


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
            rx_count = payload[4]
            tx_count = payload[5]
            canintf = payload[7]
            canstat = payload[8]
            eflg = payload[9]
            cnf1 = payload[10]
            return {
                "rx_count": rx_count,
                "tx_count": tx_count,
                "canintf": canintf,
                "canstat": canstat,
                "eflg": eflg,
                "cnf1": cnf1,
            }
    return None


def test_speed(ser, speed_name, speed_bps, duration=5):
    print(f"\n{'='*60}")
    print(f"Testing {speed_name} kbps ({speed_bps} bps)")
    print(f"{'='*60}")

    # Set speed
    if not set_speed(ser, speed_bps):
        print("  FAILED to set speed!")
        return False

    # Verify CNF1
    debug = get_debug(ser)
    if debug:
        print(f"CNF1=0x{debug['cnf1']:02X}")

    # Start capture
    send_cmd(ser, CMD_START_CAPTURE)

    print(f"\nMonitoring for {duration} seconds... (start your playback at {speed_name} kbps)")
    print("-" * 60)

    total_rx = 0
    total_tx = 0
    errors_seen = False

    for i in range(duration * 2):
        debug = get_debug(ser)
        if debug:
            total_rx += debug["rx_count"]
            total_tx += debug["tx_count"]

            # Check for errors
            eflg = debug["eflg"]
            canintf = debug["canintf"]

            status = ""
            if canintf & 0x80:
                status += "MERRF "
                errors_seen = True
            if canintf & 0x20:
                status += "ERRIF "
            if eflg & 0x40:
                status += "RX0OVR "
            if canintf & 0x01:
                status += "RX0 "
            if canintf & 0x02:
                status += "RX1 "

            print(f"[{i*0.5:4.1f}s] queued={debug['rx_count']:3d} sent={debug['tx_count']:3d} | {status}")

        time.sleep(0.5)

    # Stop capture
    send_cmd(ser, CMD_STOP_CAPTURE)

    print("-" * 60)
    print(f"Total: {total_rx} frames queued, {total_tx} frames sent to host")

    if errors_seen:
        print("WARNING: Message errors detected - possible bitrate mismatch!")
        return False
    elif total_rx == 0:
        print("WARNING: No frames received - check wiring or playback")
        return False
    else:
        print("SUCCESS: Frames captured without errors!")
        return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    # Check if specific speed requested
    speed_arg = sys.argv[2] if len(sys.argv) > 2 else None

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print(f"Connected to {port}")

    if speed_arg:
        # Test single speed
        if speed_arg in SPEEDS:
            test_speed(ser, speed_arg, SPEEDS[speed_arg])
        else:
            print(f"Unknown speed: {speed_arg}")
            print(f"Valid speeds: {', '.join(SPEEDS.keys())}")
    else:
        # Interactive mode
        print("\nAvailable speeds: 125, 250, 500, 1000 kbps")
        print("Usage: python speed_capture_test.py COM7 <speed>")
        print("\nExample: python speed_capture_test.py COM7 500")
        print("\nOr run without speed arg to test interactively:")

        while True:
            try:
                speed_input = input("\nEnter speed to test (125/250/500/1000) or 'q' to quit: ").strip()
                if speed_input.lower() == 'q':
                    break
                if speed_input in SPEEDS:
                    test_speed(ser, speed_input, SPEEDS[speed_input])
                else:
                    print(f"Invalid speed. Choose from: {', '.join(SPEEDS.keys())}")
            except KeyboardInterrupt:
                break

    ser.close()
    print("\nDone!")


if __name__ == "__main__":
    main()
