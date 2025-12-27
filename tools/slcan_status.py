#!/usr/bin/env python3
"""
SLCAN adapter status monitoring tool.
Shows TEC/REC, error state, and TX counts.

New commands added to STM32 SLCAN:
  F  - Status flags (returns F00-F07: 01=warning, 02=passive, 04=bus-off)
  E  - Extended status (returns TEC, REC, tx_success, tx_error)
  R  - Reset CAN controller (recover from bus-off)
"""

import sys
import time
import serial


def send_cmd(ser, cmd):
    """Send command and return response"""
    ser.write(f'{cmd}\r'.encode())
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(ser.in_waiting)
    return resp.decode('ascii', errors='replace').strip()


def get_status(ser):
    """Get status flags (F command)"""
    resp = send_cmd(ser, 'F')
    if resp.startswith('F') and len(resp) >= 3:
        status = int(resp[1:3], 16)
        flags = []
        if status & 0x01: flags.append("WARNING")
        if status & 0x02: flags.append("PASSIVE")
        if status & 0x04: flags.append("BUS-OFF")
        if not flags: flags.append("ACTIVE")
        return status, flags
    return None, ["UNKNOWN"]


def get_extended_status(ser):
    """Get extended status (E command): TEC, REC, tx_ok, tx_err"""
    resp = send_cmd(ser, 'E')
    # Format: ETTRROOOOEEEEE\r where TT=TEC, RR=REC, OOOO=tx_ok, EEEE=tx_err
    if resp.startswith('E') and len(resp) >= 13:
        tec = int(resp[1:3], 16)
        rec = int(resp[3:5], 16)
        tx_ok = int(resp[5:9], 16)
        tx_err = int(resp[9:13], 16)
        return {'tec': tec, 'rec': rec, 'tx_ok': tx_ok, 'tx_err': tx_err}
    return None


def reset_can(ser):
    """Reset CAN controller (R command)"""
    resp = send_cmd(ser, 'R')
    return '\r' in resp or resp == ''


def main():
    if len(sys.argv) < 2:
        print("Usage: slcan_status.py <COM_PORT> [monitor|reset]")
        print("  monitor - continuously show status (default)")
        print("  reset   - reset CAN controller and exit")
        return

    port = sys.argv[1]
    mode = sys.argv[2] if len(sys.argv) > 2 else "monitor"

    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
    except Exception as e:
        print(f"Error opening {port}: {e}")
        return

    time.sleep(0.3)
    ser.read(ser.in_waiting)

    if mode == "reset":
        print(f"Resetting CAN controller on {port}...")
        if reset_can(ser):
            print("Reset successful!")
        else:
            print("Reset failed!")
        ser.close()
        return

    print(f"SLCAN Status Monitor - {port}")
    print("=" * 50)
    print("Press Ctrl+C to exit\n")

    try:
        while True:
            status, flags = get_status(ser)
            ext = get_extended_status(ser)

            if ext:
                print(f"\rTEC={ext['tec']:3d} REC={ext['rec']:3d} | "
                      f"TX_OK={ext['tx_ok']:5d} TX_ERR={ext['tx_err']:5d} | "
                      f"State: {', '.join(flags):<20}", end="", flush=True)
            else:
                print(f"\rStatus: {', '.join(flags):<30}", end="", flush=True)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nStopped.")

    ser.close()


if __name__ == "__main__":
    main()
