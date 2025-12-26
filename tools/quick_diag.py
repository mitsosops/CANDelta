#!/usr/bin/env python3
"""Quick diagnostic for CANDelta mode change."""

import sys
import time
import serial

STX = 0x02
ETX = 0x03

# Commands (must match firmware/src/protocol/commands.h)
CMD_PING = 0x01
CMD_GET_VERSION = 0x04
CMD_DEBUG = 0x06
CMD_SET_MODE = 0x23
CMD_RESET_CAN = 0x27

RSP_ACK = 0x80
RSP_VERSION = 0x82
RSP_DEBUG = 0x85

MODE_NORMAL = 0x00
MODE_LISTEN_ONLY = 0x03


def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.1)  # Longer wait
    return ser.read(ser.in_waiting)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print(f"Connected to {port}")

    # Ping
    resp = send_cmd(ser, CMD_PING)
    print(f"PING: {resp.hex() if resp else 'no response'}")

    # Version
    resp = send_cmd(ser, CMD_GET_VERSION)
    print(f"VERSION: {resp.hex() if resp else 'no response'}")
    if RSP_VERSION in resp:
        idx = list(resp).index(RSP_VERSION)
        if idx + 6 <= len(resp):
            payload = resp[idx+2:idx+6]
            print(f"  Protocol: {payload[0]}, Version: {payload[1]}.{payload[2]}.{payload[3]}")

    # Read current mode
    resp = send_cmd(ser, CMD_DEBUG)
    if RSP_DEBUG in resp:
        idx = list(resp).index(RSP_DEBUG)
        if idx + 14 <= len(resp):
            payload = resp[idx + 2 : idx + 14]
            canstat = payload[8]
            mode = (canstat >> 5) & 0x07
            mode_names = {0: "NORMAL", 1: "SLEEP", 2: "LOOPBACK", 3: "LISTEN", 4: "CONFIG"}
            print(f"Initial CANSTAT: 0x{canstat:02X}, mode: {mode_names.get(mode, 'UNKNOWN')}")

    print("\nAttempting mode changes...")

    # Try to set NORMAL mode multiple times
    for attempt in range(3):
        print(f"\n--- Attempt {attempt+1} ---")

        # Set NORMAL mode
        resp = send_cmd(ser, CMD_SET_MODE, [MODE_NORMAL])
        print(f"SET_MODE(NORMAL) response: {resp.hex() if resp else 'no response'}")
        if RSP_ACK in resp:
            print("  -> ACK received")
        elif 0x81 in resp:
            print("  -> NAK received")
        else:
            print("  -> Unknown response")

        # Read mode immediately
        for i in range(3):
            resp = send_cmd(ser, CMD_DEBUG)
            if RSP_DEBUG in resp:
                idx = list(resp).index(RSP_DEBUG)
                if idx + 14 <= len(resp):
                    payload = resp[idx + 2 : idx + 14]
                    canstat = payload[8]
                    mode = (canstat >> 5) & 0x07
                    mode_names = {0: "NORMAL", 1: "SLEEP", 2: "LOOPBACK", 3: "LISTEN", 4: "CONFIG"}
                    print(f"  Read {i+1}: CANSTAT=0x{canstat:02X}, mode={mode_names.get(mode, 'UNKNOWN')}")

        time.sleep(0.5)

    ser.close()


if __name__ == "__main__":
    main()
