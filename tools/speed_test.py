#!/usr/bin/env python3
"""Test CAN bus speed changes on the fly"""

import sys
import time
import serial
import struct

STX = 0x02
ETX = 0x03
CMD_SET_SPEED = 0x20
CMD_DEBUG = 0x06
CMD_START_CAPTURE = 0x10
RSP_ACK = 0x80
RSP_NAK = 0x81
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
    if RSP_ACK in data:
        return True
    elif RSP_NAK in data:
        return False
    # Debug: show raw response
    if data:
        print(f"  Raw response: {data.hex()}")
    return None


def get_debug(ser):
    data = send_cmd(ser, CMD_DEBUG)
    if RSP_DEBUG in data:
        idx = list(data).index(RSP_DEBUG)
        if idx + 14 <= len(data):
            payload = data[idx + 2 : idx + 14]
            canintf = payload[7]
            canstat = payload[8]
            eflg = payload[9]
            cnf1 = payload[10]
            mode = (canstat >> 5) & 0x07
            mode_names = {0: "NORMAL", 1: "SLEEP", 2: "LOOPBACK", 3: "LISTEN", 4: "CONFIG"}
            return {
                "canintf": canintf,
                "canstat": canstat,
                "mode": mode_names.get(mode, f"?{mode}"),
                "eflg": eflg,
                "cnf1": cnf1,
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
    print("=" * 50)

    # Test each speed
    for name, speed in SPEEDS.items():
        print(f"\nSetting speed to {name} kbps ({speed} bps)...")
        result = set_speed(ser, speed)

        if result:
            print(f"  OK - Speed set successfully")
            # Get debug info to verify
            debug = get_debug(ser)
            if debug:
                print(f"  CNF1=0x{debug['cnf1']:02X} Mode={debug['mode']} EFLG=0x{debug['eflg']:02X}")
        elif result is False:
            print(f"  FAILED - NAK received")
        else:
            print(f"  ERROR - No response")

        time.sleep(0.5)

    # Return to 500kbps
    print("\n" + "=" * 50)
    print("Returning to 500 kbps...")
    set_speed(ser, 500000)

    # Start capture and monitor for 5 seconds
    print("\nStarting capture, monitoring for 5 seconds...")
    send_cmd(ser, CMD_START_CAPTURE)

    for i in range(10):
        debug = get_debug(ser)
        if debug:
            print(
                f"[{i*0.5:.1f}s] CANINTF=0x{debug['canintf']:02X} "
                f"Mode={debug['mode']} EFLG=0x{debug['eflg']:02X} CNF1=0x{debug['cnf1']:02X}"
            )
        time.sleep(0.5)

    ser.close()
    print("\nDone!")


if __name__ == "__main__":
    main()
