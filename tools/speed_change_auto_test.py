#!/usr/bin/env python3
"""
Automated test for on-the-fly speed change (no reset).
Controls both CANDelta (RX) and SLCAN TX adapter directly.

Test verifies:
1. At 500kbps: frames received (correct speed)
2. At 250kbps: CNF register changed (brief - avoid TX bus-off)
3. Back to 500kbps: frames received again (speed change worked)

Usage: python speed_change_auto_test.py [RX_PORT] [TX_PORT]
       python speed_change_auto_test.py COM7 COM8
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
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23  # NOT 0x22 (that's CLEAR_FILTERS!)
CMD_RESET_CAN = 0x27  # Reset and restore config (clears TEC/REC)

# CANDelta Responses
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_DEBUG = 0x85
RSP_CAN_FRAME = 0x84
RSP_ERROR_COUNTERS = 0x88

# MCP2515 Modes
MODE_NORMAL = 0x00
MODE_LISTEN_ONLY = 0x03


# ============ SLCAN TX Helper Functions ============

def slcan_init(ser, bitrate=500000):
    """Initialize SLCAN adapter"""
    ser.write(b'C\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    bitrate_map = {500000: 'S6', 250000: 'S5', 125000: 'S4'}
    cmd = bitrate_map.get(bitrate, 'S6')
    ser.write(f'{cmd}\r'.encode())
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.write(b'O\r')
    time.sleep(0.2)
    ser.read(ser.in_waiting)
    return True


def slcan_send_frame(ser, can_id, data):
    """Send a CAN frame via SLCAN"""
    cmd = f't{can_id:03X}{len(data):01X}'
    for byte in data:
        cmd += f'{byte:02X}'
    cmd += '\r'
    ser.write(cmd.encode())


def slcan_get_status(ser):
    """Get TX adapter status (requires updated firmware with E command)"""
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
    """Reset SLCAN CAN controller (recover from bus-off)"""
    ser.write(b'R\r')
    time.sleep(0.1)
    ser.read(ser.in_waiting)


def slcan_close(ser):
    """Close SLCAN channel"""
    ser.write(b'C\r')
    time.sleep(0.1)


# ============ CANDelta Helper Functions ============

def send_cmd(ser, opcode, params=None):
    """Send command to CANDelta and return response"""
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.05)
    return ser.read(ser.in_waiting)


def set_speed(ser, speed_bps):
    """Set CAN speed, return True if ACK received"""
    params = list(struct.pack("<I", speed_bps))
    data = send_cmd(ser, CMD_SET_SPEED, params)
    return RSP_ACK in data


def get_debug(ser):
    """Get debug info including CNF1 register"""
    data = send_cmd(ser, CMD_DEBUG)
    if RSP_DEBUG in data:
        idx = list(data).index(RSP_DEBUG)
        if idx + 14 <= len(data):
            payload = data[idx + 2 : idx + 14]
            return {
                "head": payload[0] | (payload[1] << 8),
                "tail": payload[2] | (payload[3] << 8),
                "rx_count": payload[4],
                "tx_count": payload[5],
                "capture": payload[6],
                "canintf": payload[7],
                "canstat": payload[8],
                "eflg": payload[9],
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


def count_frames(ser, duration_sec, clear_first=False):
    """Count CAN frames received over duration"""
    if clear_first:
        ser.read(ser.in_waiting)
    frame_count = 0
    start = time.time()
    while time.time() - start < duration_sec:
        data = ser.read(ser.in_waiting)
        frame_count += data.count(bytes([RSP_CAN_FRAME]))
        time.sleep(0.05)
    return frame_count


def tx_continuous(tx_ser, count, interval_ms=20):
    """Send frames continuously from TX adapter"""
    for i in range(count):
        data = [i & 0xFF, (i >> 8) & 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]
        slcan_send_frame(tx_ser, 0x123, data)
        time.sleep(interval_ms / 1000.0)


def main():
    rx_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    tx_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"

    print("=" * 60)
    print("AUTOMATED SPEED CHANGE TEST (No Reset)")
    print("=" * 60)
    print(f"CANDelta (RX): {rx_port}")
    print(f"SLCAN TX:      {tx_port}")
    print("=" * 60)

    # Open both ports
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

    # Initialize TX adapter
    time.sleep(0.3)
    tx_ser.read(tx_ser.in_waiting)
    slcan_init(tx_ser, 500000)
    print("TX adapter initialized at 500kbps.")

    # Check if TX adapter supports extended status
    tx_status = slcan_get_status(tx_ser)
    has_extended_status = tx_status is not None
    if has_extended_status:
        print(f"TX status: TEC={tx_status['tec']} REC={tx_status['rec']}")
    else:
        print("TX adapter: extended status not available (old firmware)")

    print()
    results = {}

    # ============ TEST 1: 500kbps (should receive frames) ============
    print("TEST 1: Set CANDelta to 500 kbps (should receive frames)")

    # Set NORMAL mode - CANDelta is only receiver, needs to ACK frames
    # (Default is LISTEN-ONLY which won't ACK, causing TX errors)
    send_cmd(rx_ser, CMD_SET_MODE, [MODE_NORMAL])

    if not set_speed(rx_ser, 500000):
        print("  FAILED: Could not set speed")
        rx_ser.close()
        tx_ser.close()
        return

    debug = get_debug(rx_ser)
    err = get_error_counters(rx_ser)
    print(f"  RX: CNF1=0x{debug['cnf1']:02X} TEC={err['tec']} REC={err['rec']} State={err['state']}")

    send_cmd(rx_ser, CMD_START_CAPTURE)
    time.sleep(0.1)
    rx_ser.read(rx_ser.in_waiting)  # Clear any pending data

    # Send frames and count received
    print("  Sending 50 frames from TX...")
    tx_continuous(tx_ser, 50, interval_ms=20)

    # Count frames (don't clear - frames already in buffer)
    frames_500k = count_frames(rx_ser, 0.5)
    results['500k_first'] = frames_500k
    print(f"  Frames received: {frames_500k}")

    if has_extended_status:
        tx_status = slcan_get_status(tx_ser)
        print(f"  TX: TEC={tx_status['tec']} REC={tx_status['rec']} ok={tx_status['tx_ok']} err={tx_status['tx_err']}")

    if frames_500k > 0:
        print("  PASS: Receiving frames at 500kbps\n")
    else:
        print("  FAIL: No frames received!\n")

    # ============ TEST 2: 250kbps (verify CNF change, brief) ============
    print("TEST 2: Set CANDelta to 250 kbps (verify register change)")
    print("        (Keeping brief to minimize TX bus errors)")

    if not set_speed(rx_ser, 250000):
        print("  FAILED: Could not set speed")
        rx_ser.close()
        tx_ser.close()
        return

    debug = get_debug(rx_ser)
    cnf1_250k = debug['cnf1']
    print(f"  RX: CNF1=0x{cnf1_250k:02X} (expect 0x41 for 250k)")
    results['250k_cnf1'] = cnf1_250k

    # Very brief pause - just verify register, don't linger at wrong speed
    time.sleep(0.2)

    if cnf1_250k == 0x41:
        print("  PASS: CNF1 register shows 250k timing\n")
    else:
        print(f"  FAIL: CNF1 unexpected (got 0x{cnf1_250k:02X})\n")

    # ============ TEST 3: Back to 500kbps (should receive again) ============
    print("TEST 3: Set CANDelta back to 500 kbps (should receive again)")

    # Check TX status before - might have errors from wrong-speed period
    if has_extended_status:
        tx_status = slcan_get_status(tx_ser)
        print(f"  TX before: TEC={tx_status['tec']} REC={tx_status['rec']}")
        if tx_status['tec'] > 127:
            print("  TX is error passive or bus-off, resetting...")
            slcan_reset(tx_ser)
            time.sleep(0.1)
            tx_status = slcan_get_status(tx_ser)
            print(f"  TX after reset: TEC={tx_status['tec']} REC={tx_status['rec']}")

    if not set_speed(rx_ser, 500000):
        print("  FAILED: Could not set speed")
        rx_ser.close()
        tx_ser.close()
        return

    debug = get_debug(rx_ser)
    err = get_error_counters(rx_ser)
    print(f"  RX: CNF1=0x{debug['cnf1']:02X} CANSTAT=0x{debug['canstat']:02X}")
    print(f"  RX: TEC={err['tec']} REC={err['rec']} State={err['state']}")

    time.sleep(0.1)
    rx_ser.read(rx_ser.in_waiting)  # Clear any pending data

    # Send more frames
    print("  Sending 50 frames from TX...")
    tx_continuous(tx_ser, 50, interval_ms=20)

    # Count frames (don't clear - frames already in buffer)
    frames_500k_final = count_frames(rx_ser, 0.5)
    results['500k_final'] = frames_500k_final
    print(f"  Frames received: {frames_500k_final}")

    if has_extended_status:
        tx_status = slcan_get_status(tx_ser)
        print(f"  TX: TEC={tx_status['tec']} REC={tx_status['rec']} ok={tx_status['tx_ok']} err={tx_status['tx_err']}")

    if frames_500k_final > 0:
        print("  PASS: Receiving frames after speed change\n")
    else:
        print("  FAIL: No frames after changing back!\n")

    # Cleanup
    send_cmd(rx_ser, CMD_STOP_CAPTURE)
    slcan_close(tx_ser)
    rx_ser.close()
    tx_ser.close()

    # ============ SUMMARY ============
    print("=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    print(f"  500 kbps (initial): {results['500k_first']} frames")
    print(f"  250 kbps CNF1:      0x{results['250k_cnf1']:02X} (expect 0x41)")
    print(f"  500 kbps (final):   {results['500k_final']} frames")
    print()

    passed = (
        results['500k_first'] > 0 and
        results['250k_cnf1'] == 0x41 and
        results['500k_final'] > 0
    )

    if passed:
        print("*** TEST PASSED: Speed change on-the-fly works! ***")
    else:
        if results['500k_first'] == 0:
            print("FAILED: No frames at initial 500k")
        elif results['250k_cnf1'] != 0x41:
            print("FAILED: CNF1 did not change to 250k value")
        elif results['500k_final'] == 0:
            print("FAILED: No frames after speed change")

    print("=" * 60)


if __name__ == "__main__":
    main()
