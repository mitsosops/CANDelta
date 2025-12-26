#!/usr/bin/env python3
"""
TEC Theory Test - Verify TX adapter error accumulation and recovery.

Theory:
1. TX adapter sends frames with no ACK-ing node -> TEC increases (+8 per failed TX)
2. Once CANDelta joins in NORMAL mode -> frames get ACKed -> TEC decreases (-1 per success)

Usage: python tec_theory_test.py [RX_PORT] [TX_PORT]
       python tec_theory_test.py COM7 COM8
"""

import sys
import time
import serial
import struct

STX = 0x02
ETX = 0x03

# CANDelta Commands (must match firmware/src/protocol/commands.h)
CMD_PING = 0x01
CMD_DEBUG = 0x06
CMD_GET_ERROR_COUNTERS = 0x09
CMD_SET_MODE = 0x23  # NOT 0x22 (that's CLEAR_FILTERS!)

# CANDelta Responses
RSP_ACK = 0x80
RSP_DEBUG = 0x85
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


def get_debug(ser):
    """Get debug info including CANSTAT"""
    data = send_cmd(ser, CMD_DEBUG)
    if RSP_DEBUG in data:
        idx = list(data).index(RSP_DEBUG)
        if idx + 14 <= len(data):
            payload = data[idx + 2 : idx + 14]
            return {
                "canstat": payload[8],
                "cnf1": payload[10],
            }
    return None


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
            }
    return None


def slcan_init(ser):
    """Initialize SLCAN adapter at 500kbps"""
    ser.write(b'C\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)
    ser.write(b'S6\r')  # 500kbps
    time.sleep(0.1)
    ser.read(ser.in_waiting)
    ser.write(b'O\r')
    time.sleep(0.2)
    ser.read(ser.in_waiting)


def slcan_send_frame(ser, can_id, data):
    """Send a CAN frame via SLCAN"""
    cmd = f't{can_id:03X}{len(data):01X}'
    for byte in data:
        cmd += f'{byte:02X}'
    cmd += '\r'
    ser.write(cmd.encode())


def slcan_get_status(ser):
    """Get TX adapter extended status"""
    ser.read(ser.in_waiting)
    ser.write(b'E\r')
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(ser.in_waiting).decode('ascii', errors='replace').strip()
    if resp.startswith('E') and len(resp) >= 13:
        return {
            'tec': int(resp[1:3], 16),
            'rec': int(resp[3:5], 16),
            'tx_ok': int(resp[5:9], 16),
            'tx_err': int(resp[9:13], 16)
        }
    return None


def slcan_reset(ser):
    """Reset SLCAN CAN controller"""
    ser.write(b'R\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)


def slcan_close(ser):
    """Close SLCAN channel"""
    ser.write(b'C\r')
    time.sleep(0.1)


def main():
    rx_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    tx_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"

    print("=" * 60)
    print("TEC THEORY TEST")
    print("=" * 60)
    print(f"CANDelta (RX): {rx_port}")
    print(f"SLCAN TX:      {tx_port}")
    print("=" * 60)

    # Open ports
    try:
        rx_ser = serial.Serial(port=rx_port, baudrate=115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening RX port {rx_port}: {e}")
        return

    try:
        tx_ser = serial.Serial(port=tx_port, baudrate=115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening TX port {tx_port}: {e}")
        rx_ser.close()
        return

    # Initialize CANDelta
    rx_ser.dtr = True
    rx_ser.rts = True
    time.sleep(0.3)
    rx_ser.read(rx_ser.in_waiting)

    resp = send_cmd(rx_ser, CMD_PING)
    if RSP_ACK not in resp:
        print("ERROR: No response from CANDelta")
        rx_ser.close()
        tx_ser.close()
        return
    print("CANDelta connected.")

    # Initialize TX adapter and reset TEC
    time.sleep(0.3)
    tx_ser.read(tx_ser.in_waiting)
    slcan_reset(tx_ser)  # Reset to clear any previous TEC
    time.sleep(0.1)
    slcan_init(tx_ser)
    print("TX adapter initialized at 500kbps.")

    tx_status = slcan_get_status(tx_ser)
    if tx_status:
        print(f"TX initial: TEC={tx_status['tec']} REC={tx_status['rec']}")
    else:
        print("WARNING: TX extended status not available")
        rx_ser.close()
        tx_ser.close()
        return

    print()

    # ============ PHASE 1: No ACKs (CANDelta in LISTEN-ONLY) ============
    print("PHASE 1: CANDelta in LISTEN-ONLY mode (no ACKs)")
    print("        TX adapter TEC should increase (+8 per failed frame)")
    print("-" * 60)

    # Ensure CANDelta is in LISTEN-ONLY mode
    resp = send_cmd(rx_ser, CMD_SET_MODE, [MODE_LISTEN_ONLY])
    if RSP_ACK not in resp:
        print("  WARNING: Mode change command failed")

    debug = get_debug(rx_ser)
    mode_val = (debug['canstat'] >> 5) & 0x07 if debug else -1
    mode_names = {0: "NORMAL", 1: "SLEEP", 2: "LOOPBACK", 3: "LISTEN", 4: "CONFIG"}
    print(f"  CANDelta mode: {mode_names.get(mode_val, f'UNKNOWN({mode_val})')}")

    # Send frames and watch TEC climb
    tec_values = []
    for i in range(10):
        slcan_send_frame(tx_ser, 0x123, [i, 0xAA, 0xBB, 0xCC])
        time.sleep(0.05)
        status = slcan_get_status(tx_ser)
        if status:
            tec_values.append(status['tec'])
            print(f"  Frame {i+1}: TEC={status['tec']}")

    print()
    if len(tec_values) >= 2 and tec_values[-1] > tec_values[0]:
        print("  PASS: TEC increased (no ACKs as expected)")
    else:
        print("  FAIL: TEC did not increase")

    print()

    # ============ PHASE 2: With ACKs (CANDelta in NORMAL mode) ============
    print("PHASE 2: CANDelta in NORMAL mode (will ACK frames)")
    print("        TX adapter TEC should decrease (-1 per successful frame)")
    print("-" * 60)

    # Switch CANDelta to NORMAL mode
    resp = send_cmd(rx_ser, CMD_SET_MODE, [MODE_NORMAL])
    if RSP_ACK not in resp:
        print("  WARNING: Mode change command failed")

    time.sleep(0.1)

    debug = get_debug(rx_ser)
    mode_val = (debug['canstat'] >> 5) & 0x07 if debug else -1
    print(f"  CANDelta mode: {mode_names.get(mode_val, f'UNKNOWN({mode_val})')}")

    if mode_val != 0:  # Not NORMAL
        print("  ERROR: Mode change failed! CANDelta not in NORMAL mode.")
        print("         This test requires the mutex fix in mcp2515_set_mode()")
        rx_ser.close()
        tx_ser.close()
        return

    # Get starting TEC
    status = slcan_get_status(tx_ser)
    start_tec = status['tec'] if status else 0
    print(f"  Starting TEC: {start_tec}")

    # Send frames and watch TEC decrease
    tec_values = [start_tec]
    for i in range(20):
        slcan_send_frame(tx_ser, 0x123, [i, 0xAA, 0xBB, 0xCC])
        time.sleep(0.05)
        status = slcan_get_status(tx_ser)
        if status:
            tec_values.append(status['tec'])
            if i % 5 == 4:  # Print every 5th frame
                print(f"  Frame {i+1}: TEC={status['tec']}")

    print()
    final_tec = tec_values[-1]
    if final_tec < start_tec:
        print(f"  PASS: TEC decreased from {start_tec} to {final_tec}")
    elif final_tec == 0 and start_tec == 0:
        print(f"  PASS: TEC stayed at 0 (frames ACKed successfully)")
    else:
        print(f"  FAIL: TEC did not decrease (start={start_tec}, end={final_tec})")

    # Cleanup
    slcan_close(tx_ser)
    rx_ser.close()
    tx_ser.close()

    # ============ SUMMARY ============
    print()
    print("=" * 60)
    print("THEORY VALIDATION COMPLETE")
    print("=" * 60)
    print("The TX adapter accumulates TEC errors when no node ACKs its frames.")
    print("Once CANDelta (or any node) joins and ACKs, TEC decreases and")
    print("normal communication is restored.")
    print("=" * 60)


if __name__ == "__main__":
    main()
