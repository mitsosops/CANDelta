#!/usr/bin/env python3
"""
Filter Change Test - Test CANDelta filter changes on the fly.

This test runs in a single uninterrupted flow:
1. No filters (accept all) - baseline
2. Set RX0 filter for specific ID (0x208)
3. Remove RX0, set RX1 filter for different ID (0x210)
4. Set same filter on both RX0 and RX1
5. Clear filters back to accept all (with rollover)

Usage: python filter_test.py [DELTA_PORT] [STM32_PORT]
       python filter_test.py COM7 COM8
"""

import sys
import time
import serial
import struct
import subprocess
import threading

# CANDelta protocol
STX = 0x02
ETX = 0x03
CMD_PING = 0x01
CMD_GET_STATUS = 0x05
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_SET_SPEED = 0x20
CMD_SET_FILTER = 0x21
CMD_CLEAR_FILTERS = 0x22
CMD_SET_MODE = 0x23
CMD_SET_MASK = 0x25
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_CAN_FRAME = 0x84
RSP_STATUS = 0x83
MODE_NORMAL = 0x00
MODE_LISTEN = 0x03

# Test IDs from TRC file (0x208-0x21E range)
TEST_ID_1 = 0x208  # First filter ID
TEST_ID_2 = 0x210  # Second filter ID

def send_cmd(ser, opcode, params=None):
    if params is None:
        params = []
    packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])
    ser.write(packet)
    ser.flush()
    time.sleep(0.03)
    return ser.read(ser.in_waiting)

def set_filter(ser, filter_num, can_id, extended=False):
    """Set acceptance filter"""
    params = [filter_num]
    params.extend(list(struct.pack("<I", can_id)))  # id (little-endian)
    params.append(1 if extended else 0)  # extended flag
    resp = send_cmd(ser, CMD_SET_FILTER, params)
    return RSP_ACK in resp

def set_mask(ser, mask_num, mask_val, extended=False):
    """Set acceptance mask"""
    params = [mask_num]
    params.extend(list(struct.pack("<I", mask_val)))  # mask (little-endian)
    params.append(1 if extended else 0)  # extended flag
    resp = send_cmd(ser, CMD_SET_MASK, params)
    return RSP_ACK in resp

def clear_filters(ser):
    """Clear all filters (accept all mode)"""
    resp = send_cmd(ser, CMD_CLEAR_FILTERS)
    return RSP_ACK in resp

def count_frames_by_id(data):
    """Count received frames and group by ID"""
    id_counts = {}
    total = 0
    i = 0
    while i < len(data):
        if data[i] == STX and i + 2 < len(data):
            opcode = data[i + 1]
            length = data[i + 2]
            if opcode == RSP_CAN_FRAME and i + 3 + length <= len(data):
                # Parse CAN frame: timestamp(8) + id(4) + flags(1) + dlc(1) + data
                payload = data[i + 3:i + 3 + length]
                if len(payload) >= 14:
                    can_id = struct.unpack("<I", payload[8:12])[0]
                    id_counts[can_id] = id_counts.get(can_id, 0) + 1
                    total += 1
                i += 3 + length + 1  # Skip past ETX
                continue
        i += 1
    return total, id_counts

def monitor_rx(ser, duration, label):
    """Monitor received frames for a duration"""
    print(f"\n  [{label}] Monitoring for {duration}s...")

    # Clear buffer
    ser.read(ser.in_waiting)

    start = time.time()
    all_data = b''

    while time.time() - start < duration:
        data = ser.read(ser.in_waiting)
        if data:
            all_data += data
        time.sleep(0.1)

    total, id_counts = count_frames_by_id(all_data)

    print(f"      Total frames: {total}")
    if id_counts:
        print(f"      By ID: ", end="")
        sorted_ids = sorted(id_counts.keys())
        parts = [f"0x{id:03X}:{cnt}" for id, cnt in sorted(id_counts.items())]
        # Show first 5 and summarize rest
        if len(parts) > 5:
            print(", ".join(parts[:5]) + f", ... (+{len(parts)-5} more IDs)")
        else:
            print(", ".join(parts))

    return total, id_counts

def main():
    delta_port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    stm32_port = sys.argv[2] if len(sys.argv) > 2 else "COM8"
    trc_file = r"C:\Users\Mitsos\Desktop\maxxecu_opel2.canhacker.trc"

    print("=" * 70)
    print("FILTER CHANGE ON-THE-FLY TEST")
    print("=" * 70)
    print(f"CANDelta: {delta_port}")
    print(f"STM32 TX: {stm32_port}")
    print(f"Test IDs: 0x{TEST_ID_1:03X}, 0x{TEST_ID_2:03X}")
    print("=" * 70)

    # Start TRC playback in background
    print("\nStarting TRC playback on STM32...")
    playback = subprocess.Popen(
        ["python", "tools/trc_playback.py", trc_file, stm32_port, "1.0", "loop"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=r"C:\Users\Mitsos\source\repos\CANDelta"
    )
    time.sleep(3)  # Let playback stabilize

    # Open CANDelta
    delta = serial.Serial(port=delta_port, baudrate=115200, timeout=0.5)
    delta.dtr = True
    delta.rts = True
    time.sleep(0.3)
    delta.read(delta.in_waiting)

    # Verify connection
    resp = send_cmd(delta, CMD_PING)
    if RSP_ACK not in resp:
        print("ERROR: CANDelta not responding")
        playback.terminate()
        return

    # Set 500k LISTEN mode
    send_cmd(delta, CMD_SET_SPEED, list(struct.pack("<I", 500000)))
    send_cmd(delta, CMD_SET_MODE, [MODE_LISTEN])
    print("CANDelta: 500kbps LISTEN mode")

    # Start capture
    send_cmd(delta, CMD_START_CAPTURE)
    print("Capture started.\n")

    try:
        # ================================================================
        # TEST 1: No filters (accept all) - baseline
        # ================================================================
        print("-" * 70)
        print("TEST 1: No filters (accept all) - BASELINE")
        print("-" * 70)
        total1, ids1 = monitor_rx(delta, 3, "Accept All")

        if total1 == 0:
            print("  ERROR: No frames received - check connection!")
            raise Exception("No frames received")

        expected_ids = len(ids1)
        print(f"  Result: {total1} frames, {expected_ids} unique IDs")

        # ================================================================
        # TEST 2: Set RX0 filter for TEST_ID_1 (0x208)
        # ================================================================
        print("\n" + "-" * 70)
        print(f"TEST 2: Set RX0 filter for 0x{TEST_ID_1:03X} only")
        print("-" * 70)

        # Need to set BOTH masks to avoid rollover accepting all
        # (RX1's default mask=0 means "accept all")
        if set_mask(delta, 0, 0x7FF, False):
            print(f"  Set mask0 = 0x7FF (exact match)")
        else:
            print("  ERROR: Failed to set mask0")

        if set_mask(delta, 1, 0x7FF, False):
            print(f"  Set mask1 = 0x7FF (block rollover mismatches)")
        else:
            print("  ERROR: Failed to set mask1")

        # Set filter0 for TEST_ID_1 (RX0)
        if set_filter(delta, 0, TEST_ID_1, False):
            print(f"  Set filter0 = 0x{TEST_ID_1:03X}")
        else:
            print(f"  ERROR: Failed to set filter0")

        # Set filter3 to same ID (RX1 - for rollover)
        if set_filter(delta, 3, TEST_ID_1, False):
            print(f"  Set filter3 = 0x{TEST_ID_1:03X} (for rollover)")
        else:
            print(f"  ERROR: Failed to set filter3")

        total2, ids2 = monitor_rx(delta, 3, f"RX0=0x{TEST_ID_1:03X}")

        # Verify only TEST_ID_1 received
        if len(ids2) == 1 and TEST_ID_1 in ids2:
            print(f"  Result: PASS - Only 0x{TEST_ID_1:03X} received ({ids2[TEST_ID_1]} frames)")
        elif len(ids2) == 0:
            print(f"  Result: No frames - filter may be too restrictive or ID not in stream")
        else:
            print(f"  Result: UNEXPECTED - Got {len(ids2)} IDs: {list(ids2.keys())}")

        # ================================================================
        # TEST 3: Set different filter - RX0=0x210, RX1=0x210
        # ================================================================
        print("\n" + "-" * 70)
        print(f"TEST 3: Switch to 0x{TEST_ID_2:03X} on both buffers")
        print("-" * 70)

        # Clear to reset
        if clear_filters(delta):
            print("  Cleared all filters")

        # Set both masks
        set_mask(delta, 0, 0x7FF, False)
        set_mask(delta, 1, 0x7FF, False)
        print("  Set mask0 and mask1 = 0x7FF")

        # Set both filters for TEST_ID_2
        set_filter(delta, 0, TEST_ID_2, False)
        set_filter(delta, 3, TEST_ID_2, False)
        print(f"  Set filter0 and filter3 = 0x{TEST_ID_2:03X}")

        total3, ids3 = monitor_rx(delta, 3, f"RX1=0x{TEST_ID_2:03X}")

        if len(ids3) == 1 and TEST_ID_2 in ids3:
            print(f"  Result: PASS - Only 0x{TEST_ID_2:03X} received ({ids3[TEST_ID_2]} frames)")
        elif len(ids3) == 0:
            print(f"  Result: No frames - filter may need mask1 set properly")
        else:
            print(f"  Result: UNEXPECTED - Got {len(ids3)} IDs: {list(ids3.keys())}")

        # ================================================================
        # TEST 4: Set same filter on BOTH RX0 and RX1
        # ================================================================
        print("\n" + "-" * 70)
        print(f"TEST 4: Set 0x{TEST_ID_1:03X} on both RX0 and RX1")
        print("-" * 70)

        # Clear first
        clear_filters(delta)

        # Set both masks
        set_mask(delta, 0, 0x7FF, False)
        set_mask(delta, 1, 0x7FF, False)
        print("  Set mask0 and mask1 = 0x7FF")

        # Set filter0 (RX0) and filter3 (RX1) for same ID
        set_filter(delta, 0, TEST_ID_1, False)
        set_filter(delta, 3, TEST_ID_1, False)
        print(f"  Set filter0 and filter3 = 0x{TEST_ID_1:03X}")

        total4, ids4 = monitor_rx(delta, 3, f"Both=0x{TEST_ID_1:03X}")

        if len(ids4) == 1 and TEST_ID_1 in ids4:
            print(f"  Result: PASS - Only 0x{TEST_ID_1:03X} received ({ids4[TEST_ID_1]} frames)")
        else:
            print(f"  Result: Got {len(ids4)} IDs: {list(ids4.keys())}")

        # ================================================================
        # TEST 5: Clear filters - back to accept all with rollover
        # ================================================================
        print("\n" + "-" * 70)
        print("TEST 5: Clear filters - back to accept all (verify rollover)")
        print("-" * 70)

        if clear_filters(delta):
            print("  Cleared all filters")

        total5, ids5 = monitor_rx(delta, 3, "Accept All (restored)")

        # Should see all IDs again (similar to baseline)
        if len(ids5) >= expected_ids - 2:  # Allow small variance
            print(f"  Result: PASS - Receiving {len(ids5)} IDs (baseline was {expected_ids})")
        else:
            print(f"  Result: WARNING - Only {len(ids5)} IDs (expected ~{expected_ids})")

        print(f"  Rollover active: {'YES' if total5 > 0 else 'NO'}")

        # ================================================================
        # SUMMARY
        # ================================================================
        print("\n" + "=" * 70)
        print("SUMMARY")
        print("=" * 70)
        print(f"  Test 1 (Accept All):     {total1:5d} frames, {len(ids1):2d} IDs")
        print(f"  Test 2 (RX0=0x{TEST_ID_1:03X}):    {total2:5d} frames, {len(ids2):2d} IDs")
        print(f"  Test 3 (RX1=0x{TEST_ID_2:03X}):    {total3:5d} frames, {len(ids3):2d} IDs")
        print(f"  Test 4 (Both=0x{TEST_ID_1:03X}):   {total4:5d} frames, {len(ids4):2d} IDs")
        print(f"  Test 5 (Restored All):   {total5:5d} frames, {len(ids5):2d} IDs")

        all_pass = (
            total1 > 0 and len(ids1) > 1 and  # Baseline works
            len(ids2) <= 1 and  # Filter reduced to 1 or 0 IDs
            len(ids3) <= 1 and  # Filter reduced to 1 or 0 IDs
            len(ids4) <= 1 and  # Filter reduced to 1 or 0 IDs
            len(ids5) >= expected_ids - 2  # Restored to accept all
        )

        if all_pass:
            print("\n*** ALL TESTS PASSED ***")
        else:
            print("\n*** SOME TESTS MAY NEED REVIEW ***")

    finally:
        # Stop capture
        send_cmd(delta, CMD_STOP_CAPTURE)
        delta.close()

        # Stop playback
        print("\nStopping playback...")
        playback.terminate()
        try:
            playback.wait(timeout=2)
        except:
            playback.kill()

    print("=" * 70)

if __name__ == "__main__":
    main()
