#!/usr/bin/env python3
"""Debug buffer state"""

import sys
import time
import serial

STX = 0x02
ETX = 0x03
CMD_DEBUG = 0x06
CMD_START_CAPTURE = 0x10
RSP_DEBUG = 0x85


def send_cmd(ser, opcode):
    packet = bytes([STX, opcode, 0, ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    print(f"Connected to {port}")
    print("Starting capture...")
    send_cmd(ser, CMD_START_CAPTURE)

    print("\nPolling buffer state every 500ms for 10 seconds...")
    print("-" * 50)

    for i in range(20):
        data = send_cmd(ser, CMD_DEBUG)

        if RSP_DEBUG in data:
            idx = list(data).index(RSP_DEBUG)
            if idx + 14 <= len(data):
                payload = data[idx+2:idx+14]
                head = payload[0] | (payload[1] << 8)
                tail = payload[2] | (payload[3] << 8)
                rx_count = payload[4]
                tx_count = payload[5]
                capture = payload[6]
                canintf = payload[7]
                canstat = payload[8]
                eflg = payload[9]

                mode = (canstat >> 5) & 0x07
                mode_names = {0: "NORMAL", 1: "SLEEP", 2: "LOOPBACK", 3: "LISTEN", 4: "CONFIG"}
                mode_str = mode_names.get(mode, f"?{mode}")

                cnf1 = payload[10]
                print(f"[{i*0.5:4.1f}s] q={rx_count} s={tx_count} | CANINTF=0x{canintf:02X} CANSTAT=0x{canstat:02X}({mode_str}) EFLG=0x{eflg:02X} CNF1=0x{cnf1:02X}")

        time.sleep(0.5)

    ser.close()


if __name__ == "__main__":
    main()
