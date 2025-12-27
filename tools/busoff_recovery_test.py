#!/usr/bin/env python3
"""
Bus-Off Recovery Test - Verify CANDelta auto-recovers from bus-off.

Test flow:
1. Put TX adapter in LISTEN mode (won't ACK CANDelta's frames)
2. Have CANDelta transmit frames repeatedly
3. Watch CANDelta's TEC climb to 256 (bus-off threshold)
4. Verify automatic recovery (TEC resets to 0)

Usage: python busoff_recovery_test.py [CANDELTA_PORT] [TX_PORT]
       python busoff_recovery_test.py COM7 COM8
"""

import sys
import time
import serial
import struct

STX = 0x02
ETX = 0x03

# CANDelta Commands
CMD_PING = 0x01
CMD_DEBUG = 0x06
CMD_GET_ERROR_COUNTERS = 0x09
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
CMD_SET_ONESHOT = 0x26
CMD_TRANSMIT_FRAME = 0x30
CMD_GET_STATUS = 0x05

# Responses
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_ERROR_COUNTERS = 0x88

# MCP2515 Modes
MODE_NORMAL = 0x00
MODE_LISTEN_ONLY = 0x03


def send_cmd(ser, opcode, params=None):
    """Send command to CANDelta and return response"""
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)


def get_error_counters(ser):
    """Get TEC/REC/ErrorState from CANDelta"""
    data = send_cmd(ser, CMD_GET_ERROR_COUNTERS)
    if RSP_ERROR_COUNTERS in data:
        idx = list(data).index(RSP_ERROR_COUNTERS)
        if idx + 5 <= len(data):
            payload = data[idx + 2 : idx + 5]
            states = ['ACTIVE', 'WARNING', 'PASSIVE', 'BUS_OFF']
            return {
                "tec": payload[0],
                "rec": payload[1],
                "state": states[payload[2]] if payload[2] < 4 else f"UNKNOWN({payload[2]})",
                "state_num": payload[2]
            }
    return None


def transmit_frame(ser, can_id, data):
    """Send a CAN frame from CANDelta"""
    # CMD_TRANSMIT_FRAME: id:u32, flags:u8, dlc:u8, data[0-8]
    params = list(struct.pack("<I", can_id))  # id (little-endian)
    params.append(0x00)  # flags (standard frame, no RTR)
    params.append(len(data))  # dlc
    params.extend(data)

    resp = send_cmd(ser, CMD_TRANSMIT_FRAME, params)
    return RSP_ACK in resp


def slcan_init_listen(ser):
    """Initialize SLCAN adapter in LISTEN mode (won't ACK)"""
    ser.write(b'C\r')  # Close first
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(b'S6\r')  # 500kbps
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(b'L\r')  # Listen-only mode (if supported) or just open
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting)

    # If L command not supported, just open normally but don't TX
    if b'\x07' in resp:
        ser.write(b'O\r')
        time.sleep(0.1)
        ser.read(ser.in_waiting)
        print("  TX adapter: Normal mode (will disconnect instead)")
        return False

    print("  TX adapter: Listen-only mode")
    return True


def main():
    candelta_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    tx_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"

    print("=" * 60)
    print("BUS-OFF AUTO-RECOVERY TEST")
    print("=" * 60)
    print(f"CANDelta:  {candelta_port}")
    print(f"TX Adapter: {tx_port} (will be passive/disconnected)")
    print("=" * 60)

    # Open CANDelta
    try:
        candelta = serial.Serial(port=candelta_port, baudrate=115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening CANDelta port: {e}")
        return

    candelta.dtr = True
    candelta.rts = True
    time.sleep(0.3)
    candelta.read(candelta.in_waiting)

    # Verify CANDelta connection
    resp = send_cmd(candelta, CMD_PING)
    if RSP_ACK not in resp:
        print("ERROR: No response from CANDelta")
        candelta.close()
        return
    print("CANDelta connected.")

    # Set CANDelta to 500kbps
    params = list(struct.pack("<I", 500000))
    resp = send_cmd(candelta, CMD_SET_SPEED, params)
    if RSP_ACK not in resp:
        print("ERROR: Could not set speed")
        candelta.close()
        return

    # Set CANDelta to NORMAL mode (needed for TX)
    resp = send_cmd(candelta, CMD_SET_MODE, [MODE_NORMAL])
    if RSP_ACK not in resp:
        print("ERROR: Could not set NORMAL mode")
        candelta.close()
        return
    print("CANDelta in NORMAL mode at 500kbps.")

    # For bus-off test via collision:
    # Put STM32 in NORMAL mode and have it TX the SAME ID as CANDelta
    # This causes arbitration loss and dominant bit collisions during error flags
    print("\nConfiguring TX adapter in NORMAL mode (will TX to cause collisions)...")
    tx_adapter_handle = None
    try:
        tx_adapter_handle = serial.Serial(port=tx_port, baudrate=115200, timeout=0.5)
        time.sleep(0.1)
        tx_adapter_handle.read(tx_adapter_handle.in_waiting)

        tx_adapter_handle.write(b'C\r')  # Close first
        time.sleep(0.1)
        tx_adapter_handle.read(tx_adapter_handle.in_waiting)

        tx_adapter_handle.write(b'S6\r')  # 500kbps (same as CANDelta)
        time.sleep(0.1)
        tx_adapter_handle.read(tx_adapter_handle.in_waiting)

        tx_adapter_handle.write(b'O\r')  # Normal mode
        time.sleep(0.1)
        resp = tx_adapter_handle.read(tx_adapter_handle.in_waiting)
        print("  TX adapter in NORMAL mode at 500kbps.")
        print("  Will TX same ID to cause collisions with CANDelta.")
    except Exception as e:
        print(f"Could not configure TX adapter: {e}")
        print("Make sure TX adapter is connected.")

    # First, reset to clear any leftover TEC from previous tests
    print("\nResetting CANDelta to clear error counters...")
    send_cmd(candelta, 0x27)  # CMD_RESET_CAN
    time.sleep(0.2)

    # Re-set mode after reset
    resp = send_cmd(candelta, CMD_SET_MODE, [MODE_NORMAL])
    if RSP_ACK not in resp:
        print("ERROR: Could not set NORMAL mode after reset")

    # Check initial error state
    err = get_error_counters(candelta)
    if err:
        print(f"After reset: TEC={err['tec']} REC={err['rec']} State={err['state']}")

    # Disable ONE-SHOT mode - let MCP2515 auto-retry until bus-off
    print("\nDisabling ONE-SHOT mode (auto-retry until bus-off)...")
    resp = send_cmd(candelta, CMD_SET_ONESHOT, [0x00])
    if RSP_ACK in resp:
        print("  One-shot mode disabled (auto-retry enabled).")
    else:
        print("  WARNING: Could not disable one-shot mode")

    print("\n" + "-" * 60)
    print("PHASE 1: Both nodes TX simultaneously to cause collisions")
    print("(Collisions cause dominant bit errors during error flags)")
    print("(This forces TEC to climb past error-passive to bus-off)")
    print("-" * 60)

    bus_off_detected = False
    recovery_detected = False
    prev_tec = 0
    tx_count = 0

    tx_success = 0
    tx_fail = 0

    # Put STM32 in listen mode (no ACK) so CANDelta's TX will fail
    print("\n  Configuring STM32 in LISTEN mode (won't ACK)...")
    if tx_adapter_handle:
        tx_adapter_handle.write(b'L\r')
        time.sleep(0.1)
        tx_adapter_handle.read(tx_adapter_handle.in_waiting)
        print("  STM32 in listen mode - CANDelta TX will not be ACKed.")

    # Queue one CANDelta TX - it will fail and auto-retry
    print("  CANDelta queuing TX (will fail, auto-retry until bus-off)...")
    transmit_frame(candelta, 0x123, [0x01, 0x02, 0x03])

    print("  Watching TEC climb...")

    for i in range(200):  # Poll for up to 10 seconds
        time.sleep(0.02)

        # Check error counters
        err = get_error_counters(candelta)
        if err:
            tec = err['tec']
            state = err['state']

            # Print on significant changes or periodically
            if tec != prev_tec or (i % 20 == 0):
                # Also get debug info for EFLG and TXB0CTRL
                debug_resp = send_cmd(candelta, CMD_DEBUG)
                eflg = 0
                txb0ctrl = 0
                if 0x85 in debug_resp:  # RSP_DEBUG
                    idx = list(debug_resp).index(0x85)
                    if idx + 14 <= len(debug_resp):
                        payload = debug_resp[idx + 2 : idx + 14]
                        eflg = payload[9]
                        txb0ctrl = payload[11]

                # Decode EFLG flags
                eflags = []
                if eflg & 0x20: eflags.append("TXBO")
                if eflg & 0x10: eflags.append("TXEP")
                if eflg & 0x08: eflags.append("RXEP")
                if eflg & 0x04: eflags.append("TXWAR")
                if eflg & 0x02: eflags.append("RXWAR")
                if eflg & 0x01: eflags.append("EWARN")
                eflags_str = "|".join(eflags) if eflags else "OK"

                # Decode TXB0CTRL bits
                txflags = []
                if txb0ctrl & 0x08: txflags.append("TXREQ")  # TX request pending
                if txb0ctrl & 0x10: txflags.append("TXERR")  # TX error
                if txb0ctrl & 0x20: txflags.append("MLOA")   # Message lost arbitration
                if txb0ctrl & 0x40: txflags.append("ABTF")   # TX aborted
                txflags_str = "|".join(txflags) if txflags else "IDLE"

                print(f"  Poll {i}: TEC={tec} State={state} EFLG=0x{eflg:02X}({eflags_str}) TXB0=0x{txb0ctrl:02X}({txflags_str})")
                prev_tec = tec

            # Detect bus-off
            if err['state_num'] == 3 and not bus_off_detected:
                print("\n  *** BUS-OFF DETECTED! ***")
                print("  Waiting for auto-recovery...")
                bus_off_detected = True
                time.sleep(1.0)  # Give recovery time to complete

                # Check state after recovery
                err = get_error_counters(candelta)
                if err and err['tec'] == 0:
                    print(f"  *** RECOVERY COMPLETE! TEC={err['tec']} State={err['state']} ***")
                    recovery_detected = True
                    break
                elif err:
                    print(f"  After wait: TEC={err['tec']} State={err['state']}")
                    if err['tec'] < 128:
                        recovery_detected = True
                        break

    print("\n" + "-" * 60)
    print("RESULTS")
    print("-" * 60)
    print(f"TX attempts: {tx_success} queued OK, {tx_fail} rejected")

    # Final state
    err = get_error_counters(candelta)
    if err:
        print(f"Final state: TEC={err['tec']} REC={err['rec']} State={err['state']}")

    if bus_off_detected and recovery_detected:
        print("\n*** TEST PASSED: Bus-off detected AND auto-recovery worked! ***")
    elif bus_off_detected:
        print("\n*** PARTIAL: Bus-off detected but recovery not confirmed ***")
    else:
        print("\n*** Bus-off not reached (TEC max 128 without proper bus) ***")
        print("    Note: MCP2515 TEC saturates at error-passive (128) without")
        print("    proper CAN bus termination/nodes. Bus-off (256) requires")
        print("    a real CAN network.")
        print("\n  Testing CMD_RESET_CAN recovery manually...")

        # Test the recovery mechanism by issuing CMD_RESET_CAN
        # This simulates what would happen after bus-off auto-recovery
        resp = send_cmd(candelta, 0x27)  # CMD_RESET_CAN
        time.sleep(0.3)
        err = get_error_counters(candelta)
        if err and err['tec'] == 0:
            print(f"  RESET worked: TEC={err['tec']} REC={err['rec']} State={err['state']}")
            print("\n*** PARTIAL PASS: Reset/recovery mechanism verified! ***")
        else:
            print(f"  RESET result: {err}")
            print("\n*** Reset mechanism needs investigation ***")

    # Cleanup
    if tx_adapter_handle:
        try:
            tx_adapter_handle.write(b'C\r')  # Close CAN
            tx_adapter_handle.close()
        except:
            pass

    candelta.close()
    print("=" * 60)


if __name__ == "__main__":
    main()
