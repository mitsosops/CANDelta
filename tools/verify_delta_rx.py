#!/usr/bin/env python3
"""
Verify CANDelta is receiving frames from STM32.

This test:
1. Has STM32 send a few frames slowly
2. Checks if CANDelta receives them (via capture)
"""

import sys
import time
import serial
import struct

# CANDelta protocol
STX = 0x02
ETX = 0x03
CMD_PING = 0x01
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_GET_STATUS = 0x05
CMD_DEBUG = 0x06
RSP_ACK = 0x80
RSP_CAN_FRAME = 0x84
RSP_STATUS = 0x83
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

def main():
    delta_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    stm32_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"

    print("=" * 60)
    print("Verify CANDelta RX")
    print("=" * 60)

    # Open both ports
    delta = serial.Serial(port=delta_port, baudrate=115200, timeout=0.5)
    stm32 = serial.Serial(port=stm32_port, baudrate=115200, timeout=0.1)

    delta.dtr = True
    delta.rts = True
    time.sleep(0.3)
    delta.read(delta.in_waiting)
    stm32.read(stm32.in_waiting)

    # Ping CANDelta
    resp = send_cmd(delta, CMD_PING)
    if RSP_ACK not in resp:
        print("CANDelta not responding!")
        return
    print("CANDelta: Connected")

    # Set CANDelta to 500k NORMAL mode
    params = list(struct.pack("<I", 500000))
    send_cmd(delta, CMD_SET_SPEED, params)
    send_cmd(delta, CMD_SET_MODE, [MODE_NORMAL])
    print("CANDelta: 500kbps NORMAL mode")

    # Start capture
    send_cmd(delta, CMD_START_CAPTURE)
    print("CANDelta: Capture started")

    # Init STM32 fresh
    stm32.write(b'C\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)

    stm32.write(b'R\r')  # Reset CAN to clear TEC
    time.sleep(0.2)
    stm32.read(stm32.in_waiting)

    stm32.write(b'S6\r')  # 500kbps
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)

    stm32.write(b'O\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    print("STM32: Fresh init at 500kbps (TEC reset)")

    # Check STM32 TEC
    stm32.write(b'E\r')
    time.sleep(0.1)
    resp = stm32.read(stm32.in_waiting)
    print(f"STM32 Extended Status: {resp}")

    # Send 5 frames slowly
    print("\nSending 5 frames slowly...")
    for i in range(5):
        cmd = f"t{0x100+i:03X}8{'01' * 8}\r"
        stm32.write(cmd.encode())
        time.sleep(0.05)
        resp = stm32.read(stm32.in_waiting)
        status = "OK" if b'\r' in resp and b'\x07' not in resp else "ERR"
        print(f"  Frame {i+1}: ID=0x{0x100+i:03X} -> {status}")
        time.sleep(0.2)

    # Check STM32 TEC again
    stm32.write(b'E\r')
    time.sleep(0.1)
    resp = stm32.read(stm32.in_waiting)
    print(f"\nSTM32 Extended Status after TX: {resp}")

    # Read CANDelta buffer
    print("\nReading CANDelta capture buffer...")
    time.sleep(0.2)
    rx_data = delta.read(delta.in_waiting)
    print(f"Received {len(rx_data)} bytes from CANDelta")

    # Count CAN frames in response
    frame_count = rx_data.count(bytes([RSP_CAN_FRAME]))
    print(f"CAN frames received: {frame_count}")

    # Stop capture
    send_cmd(delta, CMD_STOP_CAPTURE)

    # Get CANDelta status
    resp = send_cmd(delta, CMD_GET_STATUS)
    print(f"\nCANDelta status response: {resp.hex() if resp else 'None'}")

    # Get debug info
    resp = send_cmd(delta, CMD_DEBUG)
    if resp and 0x85 in resp:
        idx = list(resp).index(0x85)
        if idx + 14 <= len(resp):
            payload = resp[idx + 2:idx + 14]
            rx_count = payload[4]
            tx_count = payload[5]
            canintf = payload[7]
            eflg = payload[9]
            print(f"\nCANDelta Debug:")
            print(f"  RX count (low byte): {rx_count}")
            print(f"  TX count (low byte): {tx_count}")
            print(f"  CANINTF: 0x{canintf:02X}")
            print(f"  EFLG: 0x{eflg:02X}")

    delta.close()
    stm32.close()
    print("\n" + "=" * 60)

if __name__ == "__main__":
    main()
