#!/usr/bin/env python3
"""
CANDelta TX Test - Test CANDelta's recovery when IT is the transmitter.

This test:
1. CANDelta transmits frames continuously
2. STM32 receives at correct speed (ACKs)
3. STM32 switches to wrong speed (no ACKs)
4. STM32 switches back (ACKs resume)
5. Verify CANDelta recovers and continues transmitting

Usage: python delta_tx_test.py [DELTA_PORT] [STM32_PORT]
       python delta_tx_test.py COM7 COM8
"""

import sys
import time
import serial
import struct
import threading

# CANDelta protocol
STX = 0x02
ETX = 0x03
CMD_PING = 0x01
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
CMD_TRANSMIT_FRAME = 0x30
CMD_GET_ERROR_COUNTERS = 0x09
CMD_DEBUG = 0x06
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_ERROR_COUNTERS = 0x88
MODE_NORMAL = 0x00

def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.02)
    return ser.read(ser.in_waiting)

def get_delta_error_counters(ser):
    """Get TEC/REC from CANDelta"""
    resp = send_cmd(ser, CMD_GET_ERROR_COUNTERS)
    if RSP_ERROR_COUNTERS in resp:
        idx = list(resp).index(RSP_ERROR_COUNTERS)
        if idx + 5 <= len(resp):
            payload = resp[idx + 2:idx + 5]
            return {'tec': payload[0], 'rec': payload[1], 'state': payload[2]}
    return None

def transmit_frame(ser, can_id, data):
    """Transmit a CAN frame from CANDelta"""
    params = list(struct.pack("<I", can_id))  # id (little-endian)
    params.append(0x00)  # flags (standard frame)
    params.append(len(data))  # dlc
    params.extend(data)
    resp = send_cmd(ser, CMD_TRANSMIT_FRAME, params)
    return RSP_ACK in resp

# Shared state
tx_stats = {'sent': 0, 'ack': 0, 'nak': 0, 'running': True}
stats_lock = threading.Lock()

def delta_tx_thread(delta):
    """Continuously transmit from CANDelta"""
    global tx_stats
    frame_id = 0x200
    data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]

    while tx_stats['running']:
        # Transmit frame
        params = list(struct.pack("<I", frame_id))
        params.append(0x00)  # flags
        params.append(len(data))  # dlc
        params.extend(data)

        packet = bytes([STX, CMD_TRANSMIT_FRAME, len(params)] + params + [ETX])
        delta.write(packet)
        delta.flush()

        time.sleep(0.005)  # ~200 fps
        resp = delta.read(delta.in_waiting)

        with stats_lock:
            tx_stats['sent'] += 1
            if RSP_ACK in resp:
                tx_stats['ack'] += 1
            elif RSP_NAK in resp:
                tx_stats['nak'] += 1

def main():
    delta_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    stm32_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"

    print("=" * 70)
    print("CANDelta TX RECOVERY TEST")
    print("=" * 70)
    print(f"CANDelta (TX): {delta_port}")
    print(f"STM32 (RX):    {stm32_port}")
    print("=" * 70)

    # Open ports
    delta = serial.Serial(port=delta_port, baudrate=115200, timeout=0.1)
    stm32 = serial.Serial(port=stm32_port, baudrate=115200, timeout=0.1)

    delta.dtr = True
    delta.rts = True
    time.sleep(0.3)
    delta.read(delta.in_waiting)
    stm32.read(stm32.in_waiting)

    # Verify CANDelta
    resp = send_cmd(delta, CMD_PING)
    if RSP_ACK not in resp:
        print("ERROR: CANDelta not responding")
        return
    print("CANDelta connected.")

    # Set CANDelta to 500k NORMAL mode
    send_cmd(delta, CMD_SET_SPEED, list(struct.pack("<I", 500000)))
    send_cmd(delta, CMD_SET_MODE, [MODE_NORMAL])
    print("CANDelta: 500kbps NORMAL mode")

    # Set STM32 to 500k NORMAL mode
    stm32.write(b'C\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    stm32.write(b'R\r')  # Reset to clear any stuck state
    time.sleep(0.2)
    stm32.read(stm32.in_waiting)
    stm32.write(b'S6\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    stm32.write(b'O\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    print("STM32: 500kbps NORMAL mode (will ACK)")

    # Check initial CANDelta error counters
    err = get_delta_error_counters(delta)
    if err:
        print(f"CANDelta initial: TEC={err['tec']} REC={err['rec']} State={err['state']}")

    # Start TX thread
    print("\nStarting CANDelta TX thread...")
    tx_stats['running'] = True
    tx_stats['sent'] = tx_stats['ack'] = tx_stats['nak'] = 0

    tx_thread = threading.Thread(target=delta_tx_thread, args=(delta,))
    tx_thread.start()
    time.sleep(1)

    print("\n" + "-" * 70)
    print("PHASE 1: STM32 at 500k (will ACK) - 5 seconds")
    print("-" * 70)
    print(f"{'Time':>6}  {'Sent':>8}  {'ACK':>8}  {'NAK':>8}  {'TEC':>4}")

    for i in range(10):
        time.sleep(0.5)
        # Can't get error counters while TX thread is using the port
        with stats_lock:
            print(f"{i*0.5:6.1f}  {tx_stats['sent']:8d}  {tx_stats['ack']:8d}  {tx_stats['nak']:8d}")

    phase1_nak = tx_stats['nak']

    print("\n" + "-" * 70)
    print("PHASE 2: STM32 switching to LISTEN mode (no ACKs) - 5 seconds")
    print("-" * 70)

    # Switch STM32 to listen mode (won't ACK)
    stm32.write(b'L\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    print("  >>> STM32 now in LISTEN mode (no ACKs)")

    for i in range(10):
        time.sleep(0.5)
        with stats_lock:
            print(f"{5+i*0.5:6.1f}  {tx_stats['sent']:8d}  {tx_stats['ack']:8d}  {tx_stats['nak']:8d}")

    phase2_nak = tx_stats['nak']

    print("\n" + "-" * 70)
    print("PHASE 3: STM32 back to NORMAL mode (ACKs resume) - 10 seconds")
    print("-" * 70)

    # Switch STM32 back to normal mode
    stm32.write(b'C\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    stm32.write(b'O\r')
    time.sleep(0.1)
    stm32.read(stm32.in_waiting)
    print("  >>> STM32 now in NORMAL mode (will ACK)")

    prev_nak = tx_stats['nak']
    stable_count = 0
    recovery_time = None

    for i in range(20):
        time.sleep(0.5)
        with stats_lock:
            curr_nak = tx_stats['nak']
            delta_nak = curr_nak - prev_nak
            status = "STABLE" if delta_nak == 0 else f"+{delta_nak}"
            print(f"{10+i*0.5:6.1f}  {tx_stats['sent']:8d}  {tx_stats['ack']:8d}  {curr_nak:8d}  {status}")

            if delta_nak == 0:
                stable_count += 1
                if recovery_time is None and stable_count >= 3:
                    recovery_time = 10 + i * 0.5
            else:
                stable_count = 0
            prev_nak = curr_nak

    # Stop TX thread
    tx_stats['running'] = False
    tx_thread.join(timeout=2)

    # Get final error counters
    time.sleep(0.1)
    delta.read(delta.in_waiting)
    err = get_delta_error_counters(delta)

    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"Total sent: {tx_stats['sent']}")
    print(f"Total ACK:  {tx_stats['ack']}")
    print(f"Total NAK:  {tx_stats['nak']}")
    if err:
        print(f"Final TEC:  {err['tec']}")
        print(f"Final REC:  {err['rec']}")
    print()
    print(f"NAKs during correct speed (phase 1): {phase1_nak}")
    print(f"NAKs during no-ACK period (phase 2): {phase2_nak - phase1_nak}")
    print(f"NAKs after recovery (phase 3):       {tx_stats['nak'] - phase2_nak}")

    if recovery_time:
        print(f"\n*** Recovery detected at {recovery_time:.1f}s ***")

    if tx_stats['nak'] - phase2_nak == 0:
        print("\n*** PASS: CANDelta recovered - no NAKs after STM32 resumed ACKing ***")
    elif stable_count >= 5:
        print("\n*** PARTIAL: CANDelta eventually stabilized ***")
    else:
        print("\n*** ISSUE: CANDelta continued getting NAKs after recovery ***")

    delta.close()
    stm32.close()
    print("=" * 70)

if __name__ == "__main__":
    main()
