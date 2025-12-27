#!/usr/bin/env python3
"""
Frame Loss Test - Controls both CANDelta RX and SLCAN TX to measure frame loss.

Usage: python frame_loss_test.py <RX_PORT> <TX_PORT> [num_frames] [interval_ms]
Example: python frame_loss_test.py COM7 COM8 200 20
"""

import sys
import time
import serial
import threading
from collections import defaultdict

# CANDelta Protocol constants
STX = 0x02
ETX = 0x03
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
RSP_ACK = 0x80
RSP_CAN_FRAME = 0x84


class CANDeltaReceiver:
    """CANDelta RX device handler"""

    def __init__(self, port):
        self.port = port
        self.ser = None
        self.frames = []
        self.running = False
        self.thread = None
        self.buffer = b''

    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=0.1)
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(0.3)
        self.ser.read(self.ser.in_waiting)

    def start_capture(self):
        # Send start command
        packet = bytes([STX, CMD_START_CAPTURE, 0, ETX])
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.1)

        # Check ACK
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            if RSP_ACK not in data:
                print(f"  Warning: No ACK from CANDelta")

        self.running = True
        self.thread = threading.Thread(target=self._receive_thread)
        self.thread.start()

    def _receive_thread(self):
        while self.running:
            if self.ser.in_waiting:
                self.buffer += self.ser.read(self.ser.in_waiting)
                self._process_buffer()
            time.sleep(0.001)

    def _process_buffer(self):
        """Process received data using length-based parsing"""
        while STX in self.buffer:
            stx_idx = self.buffer.index(STX)

            # Need at least STX + opcode + length
            if len(self.buffer) < stx_idx + 3:
                break

            # Get expected packet length from length field
            length = self.buffer[stx_idx + 2]
            expected_end = stx_idx + 3 + length  # STX + opcode + len + payload

            # Need complete packet + ETX
            if len(self.buffer) <= expected_end:
                break

            # Check for ETX at expected position
            if self.buffer[expected_end] != ETX:
                # Corrupted packet - skip this STX and try next
                self.buffer = self.buffer[stx_idx + 1:]
                continue

            packet = self.buffer[stx_idx+1:expected_end]
            self.buffer = self.buffer[expected_end+1:]

            if len(packet) >= 2:
                opcode = packet[0]
                plen = packet[1]
                payload = packet[2:2+plen]

                if opcode == RSP_CAN_FRAME and len(payload) >= 14:
                    timestamp = int.from_bytes(payload[0:8], 'little')
                    can_id = int.from_bytes(payload[8:12], 'little')
                    flags = payload[12]
                    dlc = payload[13]
                    data = payload[14:14+dlc]

                    self.frames.append({
                        'timestamp': timestamp,
                        'can_id': can_id,
                        'flags': flags,
                        'dlc': dlc,
                        'data': bytes(data)
                    })

    def stop_capture(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        # Send stop command
        packet = bytes([STX, CMD_STOP_CAPTURE, 0, ETX])
        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.1)

    def close(self):
        if self.ser:
            self.ser.close()


class SLCANTransmitter:
    """SLCAN TX device handler"""

    def __init__(self, port):
        self.port = port
        self.ser = None
        self.sent_count = 0
        self.errors = 0

    def connect(self, bitrate=500000):
        # Try 115200 first, then 1M baud
        for baud in [115200, 1000000]:
            try:
                self.ser = serial.Serial(self.port, baud, timeout=0.5)
                break
            except:
                pass

        if not self.ser:
            raise Exception(f"Failed to open {self.port}")

        time.sleep(0.3)
        self.ser.read(self.ser.in_waiting)

        # Close any open channel
        self.ser.write(b'C\r')
        time.sleep(0.1)
        self.ser.read(self.ser.in_waiting)

        # Set bitrate
        bitrate_map = {
            10000: 'S0', 20000: 'S1', 50000: 'S2', 100000: 'S3',
            125000: 'S4', 250000: 'S5', 500000: 'S6', 800000: 'S7', 1000000: 'S8',
        }
        cmd = bitrate_map.get(bitrate, 'S6')
        self.ser.write(f'{cmd}\r'.encode())
        time.sleep(0.1)
        resp = self.ser.read(self.ser.in_waiting)

        # Open channel
        self.ser.write(b'O\r')
        time.sleep(0.2)
        self.ser.read(self.ser.in_waiting)

    def send_frame(self, can_id, data):
        if can_id > 0x7FF:
            cmd = f'T{can_id:08X}{len(data):01X}'
        else:
            cmd = f't{can_id:03X}{len(data):01X}'

        for byte in data:
            cmd += f'{byte:02X}'
        cmd += '\r'

        self.ser.write(cmd.encode())
        self.sent_count += 1

    def close(self):
        if self.ser:
            self.ser.write(b'C\r')
            time.sleep(0.1)
            self.ser.close()


def run_test(rx_port, tx_port, num_frames, interval_ms):
    """Run frame loss test"""
    print(f"Frame Loss Test")
    print("=" * 60)
    print(f"  RX Port:     {rx_port} (CANDelta)")
    print(f"  TX Port:     {tx_port} (SLCAN)")
    print(f"  Frames:      {num_frames}")
    if interval_ms > 0:
        print(f"  Interval:    {interval_ms}ms ({1000/interval_ms:.0f} Hz)")
    else:
        print(f"  Interval:    0ms (MAX speed)")
    print("=" * 60)

    # Connect devices
    print("\nConnecting devices...")

    rx = CANDeltaReceiver(rx_port)
    tx = SLCANTransmitter(tx_port)

    try:
        rx.connect()
        print(f"  CANDelta ({rx_port}): Connected")
    except Exception as e:
        print(f"  CANDelta ({rx_port}): FAILED - {e}")
        return

    try:
        tx.connect(500000)
        print(f"  SLCAN ({tx_port}): Connected")
    except Exception as e:
        print(f"  SLCAN ({tx_port}): FAILED - {e}")
        rx.close()
        return

    # Start capture
    print("\nStarting capture...")
    rx.start_capture()
    time.sleep(0.5)  # Let capture settle

    # Send frames with sequence numbers in data
    print(f"Sending {num_frames} frames...")
    test_id = 0x123

    start_time = time.time()
    for seq in range(num_frames):
        # Data: [seq_low, seq_high, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]
        data = [seq & 0xFF, (seq >> 8) & 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]
        tx.send_frame(test_id, data)

        # Progress
        if (seq + 1) % 50 == 0:
            elapsed = time.time() - start_time
            rate = (seq + 1) / elapsed if elapsed > 0 else 0
            print(f"  Sent: {seq + 1}/{num_frames}, Rate: {rate:.0f} fps")

        time.sleep(interval_ms / 1000.0)

    send_time = time.time() - start_time

    # Wait for remaining frames
    print("Waiting for remaining frames...")
    time.sleep(0.5)

    # Stop capture
    rx.stop_capture()

    # Analyze results
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)

    received = len(rx.frames)
    lost = num_frames - received
    loss_pct = (lost / num_frames) * 100 if num_frames > 0 else 0

    print(f"  Sent:       {num_frames} frames")
    print(f"  Received:   {received} frames")
    print(f"  Lost:       {lost} frames ({loss_pct:.1f}%)")
    print(f"  TX Time:    {send_time:.2f}s")
    print(f"  TX Rate:    {num_frames/send_time:.0f} fps")

    # Analyze sequence gaps
    if rx.frames:
        sequences = []
        for frame in rx.frames:
            if frame['can_id'] == test_id and len(frame['data']) >= 2:
                seq = frame['data'][0] | (frame['data'][1] << 8)
                sequences.append(seq)

        if sequences:
            # Find gaps
            sequences.sort()
            gaps = []
            for i in range(1, len(sequences)):
                if sequences[i] != sequences[i-1] + 1:
                    gap_start = sequences[i-1] + 1
                    gap_end = sequences[i] - 1
                    gap_size = sequences[i] - sequences[i-1] - 1
                    gaps.append((gap_start, gap_end, gap_size))

            # Check for missing at start
            if sequences[0] > 0:
                gaps.insert(0, (0, sequences[0] - 1, sequences[0]))

            # Check for missing at end
            if sequences[-1] < num_frames - 1:
                gaps.append((sequences[-1] + 1, num_frames - 1, num_frames - 1 - sequences[-1]))

            if gaps:
                print(f"\n  Gaps detected: {len(gaps)}")
                for gap_start, gap_end, gap_size in gaps[:10]:  # Show first 10
                    if gap_size == 1:
                        print(f"    Missing: {gap_start}")
                    else:
                        print(f"    Missing: {gap_start}-{gap_end} ({gap_size} frames)")
                if len(gaps) > 10:
                    print(f"    ... and {len(gaps) - 10} more gaps")

            # Timing analysis
            if len(rx.frames) >= 2:
                deltas = []
                for i in range(1, len(rx.frames)):
                    delta = rx.frames[i]['timestamp'] - rx.frames[i-1]['timestamp']
                    deltas.append(delta)

                avg_delta = sum(deltas) / len(deltas)
                min_delta = min(deltas)
                max_delta = max(deltas)

                print(f"\n  Timestamp Analysis:")
                print(f"    Avg interval: {avg_delta:.0f} us")
                print(f"    Min interval: {min_delta:.0f} us")
                print(f"    Max interval: {max_delta:.0f} us")

    print("=" * 60)

    # Cleanup
    tx.close()
    rx.close()

    return {
        'sent': num_frames,
        'received': received,
        'lost': lost,
        'loss_pct': loss_pct
    }


def main():
    if len(sys.argv) < 3:
        print("Usage: frame_loss_test.py <RX_PORT> <TX_PORT> [num_frames] [interval_ms]")
        print("Example: frame_loss_test.py COM7 COM8 200 20")
        return 1

    rx_port = sys.argv[1]
    tx_port = sys.argv[2]
    num_frames = int(sys.argv[3]) if len(sys.argv) > 3 else 200
    interval_ms = int(sys.argv[4]) if len(sys.argv) > 4 else 20

    run_test(rx_port, tx_port, num_frames, interval_ms)
    return 0


if __name__ == "__main__":
    sys.exit(main())
