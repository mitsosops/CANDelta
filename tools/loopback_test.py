#!/usr/bin/env python3
"""
Loopback test - tests firmware without external CAN bus.
MCP2515 loopback mode echoes TX frames directly to RX internally.
If frames are lost here, the problem is in the firmware.
If loopback is 100% but external capture loses frames, the problem is the CAN bus/adapter.
"""

import sys
import time
import serial

STX = 0x02
ETX = 0x03
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_SET_SPEED = 0x20
CMD_SET_MODE = 0x23
CMD_TRANSMIT_FRAME = 0x30
RSP_CAN_FRAME = 0x84

# MCP2515 modes
MODE_NORMAL = 0
MODE_LOOPBACK = 2


def send_cmd(ser, opcode, payload=None):
    if payload is None:
        payload = []
    packet = bytes([STX, opcode, len(payload)]) + bytes(payload) + bytes([ETX])
    ser.write(packet)
    ser.flush()


def count_frames(ser, timeout=0.5):
    """Count CAN frame packets received within timeout"""
    count = 0
    state = 0
    opcode = 0
    length = 0
    payload_remaining = 0

    end_time = time.time() + timeout
    while time.time() < end_time:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue
        for b in data:
            if state == 0:  # Wait STX
                if b == STX:
                    state = 1
            elif state == 1:  # Opcode
                opcode = b
                state = 2
            elif state == 2:  # Length
                length = b
                payload_remaining = length
                if payload_remaining == 0:
                    state = 4  # Skip payload, go to ETX
                else:
                    state = 3
            elif state == 3:  # Payload
                payload_remaining -= 1
                if payload_remaining <= 0:
                    state = 4
            elif state == 4:  # ETX
                if b == ETX and opcode == RSP_CAN_FRAME:
                    count += 1
                state = 0
    return count


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    num_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    print(f"MCP2515 Loopback Test - {port}")
    print("=" * 50)
    print(f"This test bypasses the CAN bus entirely.")
    print(f"TX frames are echoed directly to RX inside the chip.")
    print("=" * 50)

    ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
    ser.dtr = True
    ser.rts = True
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    # Stop capture and reset
    send_cmd(ser, CMD_STOP_CAPTURE)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Set speed first (this resets MCP2515)
    print("\nResetting MCP2515...")
    speed = 500000
    speed_bytes = list(speed.to_bytes(4, 'little'))
    send_cmd(ser, CMD_SET_SPEED, speed_bytes)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    # Set loopback mode
    print("Setting loopback mode...")
    send_cmd(ser, CMD_SET_MODE, [MODE_LOOPBACK])
    time.sleep(0.1)
    resp = ser.read(ser.in_waiting)
    print(f"  Response: {resp.hex() if resp else '(none)'}")

    # Start capture
    send_cmd(ser, CMD_START_CAPTURE)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    print(f"Sending {num_frames} frames in loopback mode...\n")

    # Send frames and count received
    sent = 0
    received = 0

    # Send one frame at a time with delay to allow RX processing
    for i in range(num_frames):
        # Frame: ID=0x100+i, DLC=8, data=counter
        frame_id = 0x100 + (i % 256)
        frame_data = [i & 0xFF, (i >> 8) & 0xFF, 0, 0, 0, 0, 0, 0]

        # Payload: ID(4) + flags(1) + DLC(1) + data(8) = 14 bytes
        payload = list(frame_id.to_bytes(4, 'little'))
        payload.append(0x00)  # flags: standard frame
        payload.append(8)     # DLC
        payload.extend(frame_data)

        send_cmd(ser, CMD_TRANSMIT_FRAME, payload)
        sent += 1

        # Small delay to allow RX processing (simulates real CAN timing)
        time.sleep(0.005)  # 5ms = 200 fps max

        # Count received frames periodically
        if sent % 10 == 0:
            received += count_frames(ser, timeout=0.02)
            print(f"\r  Sent: {sent}, Received: {received}", end="", flush=True)

    # Final drain
    time.sleep(0.2)
    received += count_frames(ser, timeout=0.3)

    # Stop capture
    send_cmd(ser, CMD_STOP_CAPTURE)
    time.sleep(0.1)

    # Return to normal mode
    send_cmd(ser, CMD_SET_MODE, [MODE_NORMAL])
    ser.close()

    print(f"\n\n{'=' * 50}")
    print(f"RESULTS")
    print("=" * 50)
    print(f"  Sent:     {sent}")
    print(f"  Received: {received}")
    print(f"  Lost:     {sent - received}")
    print(f"  Rate:     {100 * received / sent:.1f}%")
    print("=" * 50)

    if received == sent:
        print("\n[PASS] Loopback is 100% - firmware is working correctly!")
        print("       If external capture loses frames, check:")
        print("       - CAN bus termination (120 ohm at each end)")
        print("       - Wiring and connections")
        print("       - Sending adapter configuration")
        print("       - Bus speed mismatch")
    elif received > sent * 0.95:
        print(f"\n[WARN] Minor frame loss ({sent - received} frames)")
        print("       Could be timing edge case")
    else:
        print(f"\n[FAIL] Significant frame loss in loopback!")
        print("       Problem is in the firmware, not the CAN bus")


if __name__ == "__main__":
    main()
