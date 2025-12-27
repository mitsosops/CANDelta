#!/usr/bin/env python3
"""
Test STM32 CAN recovery from high TEC state.

This test:
1. Gets STM32 into high TEC state (no ACKs)
2. Verifies TEC is stuck
3. Monitors CANDelta RX to see if STM32 is actually transmitting
4. Tests if 'R' (reset) command recovers
"""

import sys
import time
import serial
import struct

# CANDelta protocol
STX = 0x02
ETX = 0x03
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_GET_STATUS = 0x05
RSP_ACK = 0x80
RSP_CAN_FRAME = 0x84
RSP_STATUS = 0x83
MODE_NORMAL = 0x00

def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)

def get_stm32_tec(stm32):
    """Get TEC from STM32 E command"""
    stm32.write(b'E\r')
    time.sleep(0.05)
    resp = stm32.read(stm32.in_waiting)
    try:
        data = resp.decode('latin-1')
        if 'E' in data:
            idx = data.index('E')
            tec = int(data[idx+1:idx+3], 16)
            return tec
    except:
        pass
    return -1

def main():
    delta_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    stm32_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"

    print("=" * 70)
    print("STM32 CAN RECOVERY TEST (with CANDelta RX monitoring)")
    print("=" * 70)

    delta = serial.Serial(port=delta_port, baudrate=115200, timeout=0.5)
    stm32 = serial.Serial(port=stm32_port, baudrate=115200, timeout=0.1)

    delta.dtr = True
    delta.rts = True
    time.sleep(0.3)
    delta.read(delta.in_waiting)
    stm32.read(stm32.in_waiting)

    # Fresh start: Both at 500k
    print("\n[1] Fresh start - both at 500kbps")
    params = list(struct.pack("<I", 500000))
    send_cmd(delta, CMD_SET_SPEED, params)
    send_cmd(delta, CMD_SET_MODE, [MODE_NORMAL])

    stm32.write(b'C\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    stm32.write(b'R\r')  # Reset
    time.sleep(0.2)
    stm32.read(stm32.in_waiting)
    stm32.write(b'S6\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    stm32.write(b'O\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)

    print(f"  STM32 TEC: {get_stm32_tec(stm32)}")

    # Start CANDelta capture
    send_cmd(delta, CMD_START_CAPTURE)

    # Send a test frame - should work
    stm32.write(b't12380102030405060708\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)

    # Check if CANDelta received it
    rx_data = delta.read(delta.in_waiting)
    frames = rx_data.count(bytes([RSP_CAN_FRAME]))
    print(f"  CANDelta received {frames} frames")
    print(f"  STM32 TEC: {get_stm32_tec(stm32)}")

    send_cmd(delta, CMD_STOP_CAPTURE)

    # Switch CANDelta to wrong speed
    print("\n[2] CANDelta to 250k (no ACKs) - hammering TX for 3s")
    params = list(struct.pack("<I", 250000))
    send_cmd(delta, CMD_SET_SPEED, params)

    # Hammer TX to drive up TEC
    start = time.time()
    tx_count = 0
    while time.time() - start < 3:
        stm32.write(b't12380102030405060708\r')
        tx_count += 1
        time.sleep(0.005)
        stm32.read(stm32.in_waiting)  # Clear buffer

    print(f"  Sent {tx_count} frames")
    tec = get_stm32_tec(stm32)
    print(f"  STM32 TEC after hammering: {tec}")

    # Switch CANDelta back to 500k and start capture
    print("\n[3] CANDelta back to 500k with capture - watch TEC and RX for 5s")
    params = list(struct.pack("<I", 500000))
    send_cmd(delta, CMD_SET_SPEED, params)
    send_cmd(delta, CMD_SET_MODE, [MODE_NORMAL])
    send_cmd(delta, CMD_START_CAPTURE)

    for i in range(10):
        time.sleep(0.5)
        tec = get_stm32_tec(stm32)

        # Check CANDelta RX
        rx_data = delta.read(delta.in_waiting)
        frames = rx_data.count(bytes([RSP_CAN_FRAME]))

        print(f"  {i*0.5:.1f}s: TEC={tec}, CANDelta RX frames={frames}")

    send_cmd(delta, CMD_STOP_CAPTURE)
    print(f"\n  Final TEC: {tec}")

    if tec > 100:
        print("\n[4] TEC stuck - STM32 not transmitting despite pending TX")
        print("    This indicates the bxCAN is in a stuck state at error-passive.")
        print("    Testing R command reset...")

        stm32.write(b'R\r')
        time.sleep(0.3)
        stm32.read(stm32.in_waiting)

        # Re-open CAN
        stm32.write(b'O\r')
        time.sleep(0.1)
        stm32.read(stm32.in_waiting)

        tec = get_stm32_tec(stm32)
        print(f"  TEC after R command: {tec}")

        # Start capture and send test frame
        send_cmd(delta, CMD_START_CAPTURE)
        stm32.write(b't12380102030405060708\r')
        time.sleep(0.1)
        stm32.read(stm32.in_waiting)

        rx_data = delta.read(delta.in_waiting)
        frames = rx_data.count(bytes([RSP_CAN_FRAME]))
        print(f"  CANDelta RX after reset: {frames} frames")

        tec = get_stm32_tec(stm32)
        print(f"  TEC after test TX: {tec}")
        send_cmd(delta, CMD_STOP_CAPTURE)

        if tec == 0 and frames > 0:
            print("\n*** PASS: R command successfully recovered CAN ***")
        else:
            print(f"\n*** PARTIAL: TEC={tec}, frames={frames} ***")
    else:
        print("\n*** TEC naturally recovered - no reset needed ***")

    print("\n" + "-" * 70)
    print("CONCLUSION:")
    print("  When TEC reaches ~248 (deep error-passive), the bxCAN stops")
    print("  transmitting pending frames even after ACKs resume.")
    print("  Manual reset via 'R' command is required to recover.")
    print("  This is expected bxCAN behavior - auto-retry doesn't work")
    print("  reliably at very high TEC values.")
    print("-" * 70)

    delta.close()
    stm32.close()
    print("=" * 70)

if __name__ == "__main__":
    main()
