#!/usr/bin/env python3
"""Diagnose MCP2515 RX overflow issues"""

import sys
import time
import serial

STX = 0x02
ETX = 0x03
CMD_DEBUG = 0x06
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
RSP_DEBUG = 0x85


def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)


def get_debug_info(ser):
    response = send_cmd(ser, CMD_DEBUG)
    for i in range(len(response) - 3):
        if response[i] == STX and response[i + 1] == RSP_DEBUG:
            length = response[i + 2]
            if i + 3 + length + 1 <= len(response) and response[i + 3 + length] == ETX:
                payload = response[i + 3 : i + 3 + length]
                if len(payload) >= 12:
                    return {
                        "head": payload[0] | (payload[1] << 8),
                        "tail": payload[2] | (payload[3] << 8),
                        "canintf": payload[7],
                        "canstat": payload[8],
                        "eflg": payload[9],
                    }
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30

    print(f"MCP2515 Overflow Diagnostics - {port}")
    print("=" * 50)

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    # Stop and restart capture to reset MCP2515 error flags
    send_cmd(ser, CMD_STOP_CAPTURE)
    time.sleep(0.1)

    # Set speed (triggers MCP2515 reset which clears EFLG)
    speed_500k = [0x20, 0xA1, 0x07, 0x00]  # 500000 in little-endian
    send_cmd(ser, 0x20, speed_500k)  # CMD_SET_SPEED
    time.sleep(0.1)

    send_cmd(ser, CMD_START_CAPTURE)
    print("Capture started (MCP2515 reset). Monitoring for NEW overflows...\n")

    last_eflg = 0
    overflow_events = 0
    samples = 0

    start = time.time()
    try:
        while time.time() - start < duration:
            # Drain incoming CAN frames
            ser.read(ser.in_waiting)
            time.sleep(0.1)

            debug = get_debug_info(ser)
            if debug:
                samples += 1
                eflg = debug["eflg"]

                # Check for NEW overflow (flag went from 0 to 1)
                new_rx0ovr = (eflg & 0x40) != 0 and (last_eflg & 0x40) == 0
                new_rx1ovr = (eflg & 0x80) != 0 and (last_eflg & 0x80) == 0

                if new_rx0ovr or new_rx1ovr:
                    overflow_events += 1

                # Decode EFLG
                flags = []
                if eflg & 0x80: flags.append("RX1OVR")
                if eflg & 0x40: flags.append("RX0OVR")
                if eflg & 0x20: flags.append("TXBO")
                if eflg & 0x10: flags.append("TXEP")
                if eflg & 0x08: flags.append("RXEP")
                if eflg & 0x04: flags.append("TXWAR")
                if eflg & 0x02: flags.append("RXWAR")
                if eflg & 0x01: flags.append("EWARN")

                elapsed = time.time() - start
                flag_str = ", ".join(flags) if flags else "OK"

                if new_rx0ovr or new_rx1ovr:
                    status = "NEW OVERFLOW!"
                elif eflg & 0xC0:
                    status = "(sticky)"
                else:
                    status = "OK"

                print(f"{elapsed:5.1f}s | EFLG=0x{eflg:02X} | {flag_str:30} | {status}")

                last_eflg = eflg

            time.sleep(0.4)

    except KeyboardInterrupt:
        print("\nInterrupted")

    send_cmd(ser, CMD_STOP_CAPTURE)
    ser.close()

    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    print(f"  Samples: {samples}")
    print(f"  Overflow events: {overflow_events}")
    print(f"  Final EFLG: 0x{last_eflg:02X}")

    if overflow_events > 0:
        print("\n  [PROBLEM] MCP2515 RX buffer overflow detected!")
        print("  Frames are being lost at the hardware level.")
        print("\n  Possible fixes:")
        print("  1. Use interrupt-driven RX instead of polling")
        print("  2. Increase SPI speed (currently 10MHz, max 10MHz)")
        print("  3. Optimize mcp2515_receive() to read faster")
    elif last_eflg & 0xC0:
        print("\n  [PARTIAL] Overflow occurred but only once at start")
        print("  Rollover may be helping reduce frame loss")
    else:
        print("\n  [OK] No MCP2515 overflow detected")


if __name__ == "__main__":
    main()
