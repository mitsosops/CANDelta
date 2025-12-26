#!/usr/bin/env python3
"""
CANDelta Live Trace Demo
Real-time CAN capture with performance metrics and command log
"""

import sys
import time
import struct
import threading
from collections import deque
from datetime import datetime

try:
    import serial
except ImportError:
    print("Please install pyserial: pip install pyserial")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.live import Live
    from rich.text import Text
    from rich import box
except ImportError:
    print("Please install rich: pip install rich")
    sys.exit(1)

# Protocol constants
STX = 0x02
ETX = 0x03

# Commands
CMD_PING = 0x01
CMD_GET_STATUS = 0x05
CMD_GET_PERF_STATS = 0x07
CMD_START_CAPTURE = 0x10
CMD_STOP_CAPTURE = 0x11
CMD_SET_SPEED = 0x20

# Responses
RSP_ACK = 0x80
RSP_NAK = 0x81
RSP_STATUS = 0x83
RSP_CAN_FRAME = 0x84
RSP_PERF_STATS = 0x86

# Command names for display
CMD_NAMES = {
    CMD_PING: "PING",
    CMD_GET_STATUS: "GET_STATUS",
    CMD_GET_PERF_STATS: "GET_PERF_STATS",
    CMD_START_CAPTURE: "START_CAPTURE",
    CMD_STOP_CAPTURE: "STOP_CAPTURE",
    CMD_SET_SPEED: "SET_SPEED",
}


class CANDeltaDemo:
    def __init__(self, port, speed=500000):
        self.port = port
        self.speed = speed
        self.ser = None
        self.running = False
        self.start_time = None

        # Data stores
        self.frames = deque(maxlen=100)  # Last 100 frames for display
        self.frame_count = 0
        self.commands = deque(maxlen=20)  # Last 20 commands
        self.perf_stats = {"fps": 0, "peak_fps": 0, "dropped": 0, "buffer": 0}

        # Thread safety
        self.lock = threading.Lock()

        # Parser state
        self.parse_buffer = bytearray()

    def connect(self):
        """Connect to CANDelta device"""
        self.ser = serial.Serial(port=self.port, baudrate=115200, timeout=0.1)
        self.ser.dtr = True
        self.ser.rts = True
        time.sleep(0.3)
        self.ser.read(self.ser.in_waiting)  # Flush

    def send_cmd(self, opcode, params=None):
        """Send command and log it"""
        if params is None:
            params = []
        packet = bytes([STX, opcode, len(params)] + list(params) + [ETX])

        cmd_name = CMD_NAMES.get(opcode, f"0x{opcode:02X}")
        cmd_entry = {"name": cmd_name, "time": time.time(), "status": "pending"}

        with self.lock:
            self.commands.append(cmd_entry)

        self.ser.write(packet)
        self.ser.flush()
        time.sleep(0.05)

        # Read response
        response = self.ser.read(self.ser.in_waiting)

        # Check for ACK/NAK
        if RSP_ACK in response:
            cmd_entry["status"] = "ack"
        elif RSP_NAK in response:
            cmd_entry["status"] = "nak"
        elif any(r in response for r in [RSP_STATUS, RSP_PERF_STATS]):
            cmd_entry["status"] = "response"

        return response

    def set_speed(self, speed_bps):
        """Set CAN bus speed"""
        params = [
            speed_bps & 0xFF,
            (speed_bps >> 8) & 0xFF,
            (speed_bps >> 16) & 0xFF,
            (speed_bps >> 24) & 0xFF,
        ]
        return self.send_cmd(CMD_SET_SPEED, params)

    def parse_packets(self, data):
        """Parse incoming data for CAN frames and other responses"""
        self.parse_buffer.extend(data)

        while len(self.parse_buffer) >= 4:
            # Find STX
            try:
                stx_idx = self.parse_buffer.index(STX)
                if stx_idx > 0:
                    self.parse_buffer = self.parse_buffer[stx_idx:]
            except ValueError:
                self.parse_buffer.clear()
                break

            if len(self.parse_buffer) < 4:
                break

            opcode = self.parse_buffer[1]
            length = self.parse_buffer[2]
            packet_end = 3 + length + 1  # STX + opcode + len + payload + ETX

            if len(self.parse_buffer) < packet_end:
                break  # Wait for more data

            if self.parse_buffer[packet_end - 1] != ETX:
                # Invalid packet, skip this STX
                self.parse_buffer = self.parse_buffer[1:]
                continue

            # Valid packet
            payload = bytes(self.parse_buffer[3 : 3 + length])
            self.parse_buffer = self.parse_buffer[packet_end:]

            if opcode == RSP_CAN_FRAME and len(payload) >= 14:
                self.parse_can_frame(payload)
            elif opcode == RSP_PERF_STATS and len(payload) >= 13:
                self.parse_perf_stats(payload)

    def parse_perf_stats(self, payload):
        """Parse performance stats from payload"""
        with self.lock:
            self.perf_stats = {
                "fps": struct.unpack("<I", payload[0:4])[0],
                "peak_fps": struct.unpack("<I", payload[4:8])[0],
                "dropped": struct.unpack("<I", payload[8:12])[0],
                "buffer": payload[12],
            }

    def parse_can_frame(self, payload):
        """Parse a CAN frame from payload"""
        timestamp_us = struct.unpack("<Q", payload[0:8])[0]
        can_id = struct.unpack("<I", payload[8:12])[0]
        flags = payload[12]
        dlc = payload[13]
        data = list(payload[14 : 14 + dlc])

        extended = (flags & 0x01) != 0
        rtr = (flags & 0x02) != 0

        # Convert to seconds from start
        if self.start_time is None:
            self.start_time = timestamp_us
        rel_time = (timestamp_us - self.start_time) / 1_000_000.0

        frame = {
            "num": self.frame_count + 1,
            "time": rel_time,
            "id": can_id,
            "extended": extended,
            "rtr": rtr,
            "dlc": dlc,
            "data": data,
        }

        with self.lock:
            self.frames.append(frame)
            self.frame_count += 1

    def request_perf_stats(self):
        """Send perf stats request (response handled by packet parser)"""
        packet = bytes([STX, CMD_GET_PERF_STATS, 0, ETX])

        cmd_entry = {"name": "GET_PERF_STATS", "time": time.time(), "status": "response"}
        with self.lock:
            self.commands.append(cmd_entry)

        self.ser.write(packet)
        self.ser.flush()

    def create_layout(self):
        """Create the terminal layout"""
        layout = Layout()

        # Main layout: status bar at top, content below
        layout.split_column(
            Layout(name="status", size=3),
            Layout(name="main"),
        )

        # Main area: trace on left, commands on right
        layout["main"].split_row(
            Layout(name="trace", ratio=3),
            Layout(name="commands", ratio=1),
        )

        return layout

    def render_status(self):
        """Render the status bar"""
        with self.lock:
            stats = self.perf_stats.copy()
            frame_count = self.frame_count

        # Build status text
        status = Text()
        status.append(f" CANDelta @ {self.speed // 1000}kbps ", style="bold white on blue")
        status.append("  ")
        status.append(f"FPS: {stats['fps']}", style="cyan")
        status.append("  ")
        status.append(f"Peak: {stats['peak_fps']}", style="green")
        status.append("  ")

        if stats["dropped"] > 0:
            status.append(f"Dropped: {stats['dropped']}", style="bold red")
        else:
            status.append(f"Dropped: 0", style="green")

        status.append("  ")

        if stats["buffer"] > 80:
            status.append(f"Buffer: {stats['buffer']}%", style="bold red")
        elif stats["buffer"] > 50:
            status.append(f"Buffer: {stats['buffer']}%", style="yellow")
        else:
            status.append(f"Buffer: {stats['buffer']}%", style="green")

        status.append("  ")
        status.append(f"Total: {frame_count}", style="white")

        return Panel(status, box=box.SIMPLE)

    def render_trace(self):
        """Render the CAN trace in .trc format"""
        table = Table(
            box=box.SIMPLE,
            show_header=True,
            header_style="bold cyan",
            expand=True,
            padding=(0, 1),
        )

        table.add_column("#", justify="right", width=6)
        table.add_column("Time", justify="right", width=10)
        table.add_column("Dir", justify="center", width=3)
        table.add_column("ID", justify="right", width=10)
        table.add_column("DLC", justify="center", width=3)
        table.add_column("Data", justify="left")

        with self.lock:
            frames = list(self.frames)

        # Show last N frames that fit
        for frame in frames[-30:]:
            # Format ID
            if frame["extended"]:
                id_str = f"{frame['id']:08X}"
            else:
                id_str = f"{frame['id']:03X}"

            # Format data
            data_str = " ".join(f"{b:02X}" for b in frame["data"])
            if frame["rtr"]:
                data_str = "RTR"

            table.add_row(
                str(frame["num"]),
                f"{frame['time']:.4f}",
                "Rx",
                id_str,
                str(frame["dlc"]),
                data_str,
            )

        return Panel(table, title="[bold]CAN Trace (.trc format)[/bold]", border_style="blue")

    def render_commands(self):
        """Render the command sidebar"""
        table = Table(box=None, show_header=False, expand=True, padding=(0, 0))
        table.add_column("Cmd", justify="left")

        with self.lock:
            commands = list(self.commands)

        for cmd in commands[-15:]:
            if cmd["status"] == "ack" or cmd["status"] == "response":
                style = "green"
                symbol = "[OK]"
            elif cmd["status"] == "nak":
                style = "red"
                symbol = "[NAK]"
            else:
                style = "yellow"
                symbol = "[...]"

            table.add_row(Text(f"{symbol} {cmd['name']}", style=style))

        return Panel(table, title="[bold]Commands[/bold]", border_style="cyan")

    def render(self, layout):
        """Render all components"""
        layout["status"].update(self.render_status())
        layout["trace"].update(self.render_trace())
        layout["commands"].update(self.render_commands())
        return layout

    def capture_thread(self):
        """Background thread for reading CAN frames"""
        while self.running:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                self.parse_packets(data)
            time.sleep(0.01)

    def perf_thread(self):
        """Background thread for requesting perf stats"""
        while self.running:
            self.request_perf_stats()
            time.sleep(0.5)

    def run(self, duration=None):
        """Run the demo. Duration=None means run until Ctrl+C."""
        console = Console()

        console.print("[bold blue]CANDelta Live Trace Demo[/bold blue]")
        console.print(f"Connecting to {self.port}...")

        self.connect()

        # Initialize device
        console.print("Configuring device...")
        self.send_cmd(CMD_PING)
        self.set_speed(self.speed)
        self.send_cmd(CMD_START_CAPTURE)

        if duration:
            console.print(f"Capturing at {self.speed // 1000} kbps for {duration} seconds...")
        else:
            console.print(f"Capturing at {self.speed // 1000} kbps (Ctrl+C to stop)...")
        console.print("")

        time.sleep(0.5)

        # Start background threads
        self.running = True
        capture_t = threading.Thread(target=self.capture_thread, daemon=True)
        perf_t = threading.Thread(target=self.perf_thread, daemon=True)
        capture_t.start()
        perf_t.start()

        # Create layout
        layout = self.create_layout()

        try:
            with Live(layout, console=console, refresh_per_second=10, screen=True) as live:
                start = time.time()
                while duration is None or (time.time() - start < duration):
                    live.update(self.render(layout))
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.send_cmd(CMD_STOP_CAPTURE)
            self.ser.close()

        # Final summary
        console.print("\n[bold]Session Summary:[/bold]")
        console.print(f"  Total frames: {self.frame_count}")
        console.print(f"  Peak FPS: {self.perf_stats['peak_fps']}")
        console.print(f"  Dropped: {self.perf_stats['dropped']}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else None  # None = infinite

    demo = CANDeltaDemo(port, speed=500000)
    demo.run(duration)


if __name__ == "__main__":
    main()
