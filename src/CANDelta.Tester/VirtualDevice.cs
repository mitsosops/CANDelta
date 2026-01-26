using System.IO.Ports;
using CANDelta.Core.Protocol;

namespace CANDelta.Tester;

/// <summary>
/// Implements the CANDelta protocol as a virtual device.
/// </summary>
public class VirtualDevice : IDisposable
{
    private const byte Stx = 0x02;
    private const byte Etx = 0x03;

    private readonly SerialPort _port;
    private PlaybackEngine? _playbackEngine;
    private readonly bool _verbose;
    private readonly object _writeLock = new();

    private Thread? _readThread;
    private volatile bool _running;

    // Device state
    private CanSpeed _speed = CanSpeed.Speed500Kbps;
    private CanMode _mode = CanMode.Normal;
    private uint _txFrameCount;

    // Parser state
    private enum ParseState { WaitingForStx, WaitingForOpcode, WaitingForLength, ReadingPayload, WaitingForEtx }
    private ParseState _parseState = ParseState.WaitingForStx;
    private readonly byte[] _buffer = new byte[64];
    private int _bufferPos;
    private int _payloadLength;
    private int _payloadRead;

    public VirtualDevice(string portName, bool verbose)
    {
        _verbose = verbose;

        _port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One)
        {
            ReadTimeout = 100,
            WriteTimeout = 1000
        };
    }

    /// <summary>
    /// Set the playback engine after construction.
    /// </summary>
    public void SetPlaybackEngine(PlaybackEngine playbackEngine)
    {
        _playbackEngine = playbackEngine;
    }

    /// <summary>
    /// Start the virtual device, listening for commands.
    /// </summary>
    public void Start()
    {
        _port.Open();
        _running = true;

        Console.WriteLine($"Virtual CANDelta device listening on {_port.PortName}");

        _readThread = new Thread(ReadLoop)
        {
            Name = "VirtualDevice.Read",
            IsBackground = true
        };
        _readThread.Start();
    }

    /// <summary>
    /// Stop the virtual device.
    /// </summary>
    public void Stop()
    {
        _running = false;
        _playbackEngine?.Shutdown();
        _readThread?.Join(2000);

        if (_port.IsOpen)
        {
            _port.Close();
        }
    }

    public void Dispose()
    {
        Stop();
        _port.Dispose();
    }

    /// <summary>
    /// Send a CAN frame to the host. Called by PlaybackEngine.
    /// </summary>
    public void SendCanFrame(CanFrame frame)
    {
        // Build CAN_FRAME response (0x84)
        // Payload: timestamp(8) + id(4) + flags(1) + dlc(1) + data(0-8)
        var payloadLen = 14 + frame.Dlc;
        var packet = new byte[4 + payloadLen];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.CanFrame;
        packet[idx++] = (byte)payloadLen;

        // Timestamp (8 bytes LE)
        for (int i = 0; i < 8; i++)
        {
            packet[idx++] = (byte)(frame.TimestampUs >> (i * 8));
        }

        // ID (4 bytes LE)
        for (int i = 0; i < 4; i++)
        {
            packet[idx++] = (byte)(frame.Id >> (i * 8));
        }

        // Flags
        byte flags = 0;
        if (frame.IsExtended) flags |= 0x01;
        if (frame.IsRtr) flags |= 0x02;
        packet[idx++] = flags;

        // DLC
        packet[idx++] = frame.Dlc;

        // Data
        if (frame.Data != null)
        {
            for (int i = 0; i < frame.Dlc && i < frame.Data.Length; i++)
            {
                packet[idx++] = frame.Data[i];
            }
        }

        packet[idx++] = Etx;

        WritePacket(packet);
        _txFrameCount++;
    }

    private void ReadLoop()
    {
        var buffer = new byte[256];

        while (_running)
        {
            try
            {
                int bytesRead = _port.Read(buffer, 0, buffer.Length);
                for (int i = 0; i < bytesRead; i++)
                {
                    ProcessByte(buffer[i]);
                }
            }
            catch (TimeoutException)
            {
                // Normal - just continue
            }
            catch (Exception ex) when (_running)
            {
                Console.WriteLine($"Read error: {ex.Message}");
                Thread.Sleep(100);
            }
        }
    }

    private void ProcessByte(byte b)
    {
        switch (_parseState)
        {
            case ParseState.WaitingForStx:
                if (b == Stx)
                {
                    _bufferPos = 0;
                    _parseState = ParseState.WaitingForOpcode;
                }
                break;

            case ParseState.WaitingForOpcode:
                if (_bufferPos < _buffer.Length) _buffer[_bufferPos++] = b;
                _parseState = ParseState.WaitingForLength;
                break;

            case ParseState.WaitingForLength:
                if (_bufferPos < _buffer.Length) _buffer[_bufferPos++] = b;
                _payloadLength = b;
                _payloadRead = 0;
                _parseState = _payloadLength > 0 ? ParseState.ReadingPayload : ParseState.WaitingForEtx;
                break;

            case ParseState.ReadingPayload:
                if (_bufferPos < _buffer.Length) _buffer[_bufferPos++] = b;
                _payloadRead++;
                if (_payloadRead >= _payloadLength)
                {
                    _parseState = ParseState.WaitingForEtx;
                }
                break;

            case ParseState.WaitingForEtx:
                if (b == Etx)
                {
                    ProcessCommand(_buffer, _bufferPos);
                }
                _bufferPos = 0;
                _parseState = ParseState.WaitingForStx;
                break;
        }
    }

    private void ProcessCommand(byte[] data, int length)
    {
        if (length < 2) return;

        var opcode = (CommandOpcode)data[0];
        var payloadLen = data[1];
        var payload = length > 2 ? data[2..Math.Min(2 + payloadLen, length)] : Array.Empty<byte>();

        if (_verbose)
        {
            Console.WriteLine($"RX: {opcode} (len={payloadLen})");
        }

        switch (opcode)
        {
            case CommandOpcode.Ping:
                SendAck();
                break;

            case CommandOpcode.GetVersion:
                SendVersion();
                break;

            case CommandOpcode.GetStatus:
                SendStatus();
                break;

            case CommandOpcode.GetPerfStats:
                SendPerfStats();
                break;

            case CommandOpcode.GetDeviceId:
                SendDeviceId();
                break;

            case CommandOpcode.GetErrorCounters:
                SendErrorCounters();
                break;

            case CommandOpcode.StartCapture:
                _playbackEngine?.StartCapture();
                SendAck();
                Console.WriteLine("Capture started");
                break;

            case CommandOpcode.StopCapture:
                _playbackEngine?.StopCapture();
                SendAck();
                Console.WriteLine("Capture stopped");
                break;

            case CommandOpcode.SetSpeed:
                if (payload.Length >= 4)
                {
                    uint speed = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        speed |= (uint)payload[i] << (i * 8);
                    }
                    _speed = (CanSpeed)speed;
                    if (_verbose)
                    {
                        Console.WriteLine($"Speed set to {speed} bps");
                    }
                }
                SendAck();
                break;

            case CommandOpcode.SetMode:
                if (payload.Length >= 1)
                {
                    _mode = (CanMode)payload[0];
                    if (_verbose)
                    {
                        Console.WriteLine($"Mode set to {_mode}");
                    }
                }
                SendAck();
                break;

            case CommandOpcode.Debug:
                SendDebug();
                break;

            case CommandOpcode.ListCommands:
                SendCommandList();
                break;

            case CommandOpcode.GetConfig:
                SendConfig();
                break;

            case CommandOpcode.GetRegisters:
                SendRegisters();
                break;

            default:
                // Unknown or unimplemented command - just ACK
                SendAck();
                break;
        }
    }

    private void SendAck()
    {
        var packet = new byte[] { Stx, (byte)ResponseCode.Ack, 0, Etx };
        WritePacket(packet);
    }

    private void SendVersion()
    {
        // Protocol version 1, firmware v1.0.0
        var packet = new byte[] { Stx, (byte)ResponseCode.Version, 4, 1, 1, 0, 0, Etx };
        WritePacket(packet);
    }

    private void SendStatus()
    {
        // 16 bytes: protocol(1) + mode(1) + speed(4) + capture(1) + error(1) + rx(4) + tx(4)
        var packet = new byte[4 + 16];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.Status;
        packet[idx++] = 16; // payload length

        packet[idx++] = 1; // protocol version
        packet[idx++] = (byte)_mode;

        // Speed (4 bytes LE)
        var speed = (uint)_speed;
        for (int i = 0; i < 4; i++)
        {
            packet[idx++] = (byte)(speed >> (i * 8));
        }

        packet[idx++] = (byte)(_playbackEngine?.CaptureActive == true ? 1 : 0);
        packet[idx++] = 0; // error flags

        // RX frame count (4 bytes LE)
        var rxCount = _playbackEngine?.FramesSent ?? 0;
        for (int i = 0; i < 4; i++)
        {
            packet[idx++] = (byte)(rxCount >> (i * 8));
        }

        // TX frame count (4 bytes LE)
        for (int i = 0; i < 4; i++)
        {
            packet[idx++] = (byte)(_txFrameCount >> (i * 8));
        }

        packet[idx++] = Etx;
        WritePacket(packet);
    }

    private void SendPerfStats()
    {
        // 13 bytes: fps(4) + peak(4) + dropped(4) + buffer(1)
        var packet = new byte[4 + 13];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.PerfStats;
        packet[idx++] = 13;

        // Simulated stats
        uint fps = _playbackEngine?.CaptureActive == true ? 100u : 0u;
        uint peak = 200;
        uint dropped = 0;
        byte buffer = 10;

        for (int i = 0; i < 4; i++) packet[idx++] = (byte)(fps >> (i * 8));
        for (int i = 0; i < 4; i++) packet[idx++] = (byte)(peak >> (i * 8));
        for (int i = 0; i < 4; i++) packet[idx++] = (byte)(dropped >> (i * 8));
        packet[idx++] = buffer;

        packet[idx++] = Etx;
        WritePacket(packet);
    }

    private void SendDeviceId()
    {
        // 8 bytes fake device ID
        var packet = new byte[] { Stx, (byte)ResponseCode.DeviceId, 8,
            0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD, 0xBE, 0xEF, Etx };
        WritePacket(packet);
    }

    private void SendErrorCounters()
    {
        // 3 bytes: TEC(1) + REC(1) + state(1)
        var packet = new byte[] { Stx, (byte)ResponseCode.ErrorCounters, 3, 0, 0, 0, Etx };
        WritePacket(packet);
    }

    private void SendDebug()
    {
        // 12 bytes debug info
        var packet = new byte[4 + 12];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.Debug;
        packet[idx++] = 12;

        // buffer_head(2), buffer_tail(2), rx_count(1), tx_count(1),
        // capture_active(1), canintf(1), canstat(1), eflg(1), cnf1(1), txb0ctrl(1)
        packet[idx++] = 0; packet[idx++] = 0; // head
        packet[idx++] = 0; packet[idx++] = 0; // tail
        packet[idx++] = 0; // rx_count
        packet[idx++] = 0; // tx_count
        packet[idx++] = (byte)(_playbackEngine?.CaptureActive == true ? 1 : 0);
        packet[idx++] = 0; // canintf
        packet[idx++] = 0; // canstat
        packet[idx++] = 0; // eflg
        packet[idx++] = 0; // cnf1
        packet[idx++] = 0; // txb0ctrl

        packet[idx++] = Etx;
        WritePacket(packet);
    }

    private void SendCommandList()
    {
        // List of (opcode, param_count) pairs
        var commands = new (CommandOpcode, byte)[]
        {
            (CommandOpcode.Ping, 0),
            (CommandOpcode.GetVersion, 0),
            (CommandOpcode.GetStatus, 0),
            (CommandOpcode.Debug, 0),
            (CommandOpcode.GetPerfStats, 0),
            (CommandOpcode.GetDeviceId, 0),
            (CommandOpcode.GetErrorCounters, 0),
            (CommandOpcode.ListCommands, 0),
            (CommandOpcode.GetConfig, 0),
            (CommandOpcode.GetRegisters, 0),
            (CommandOpcode.StartCapture, 0),
            (CommandOpcode.StopCapture, 0),
            (CommandOpcode.SetSpeed, 1),
            (CommandOpcode.SetMode, 1),
        };

        var payloadLen = commands.Length * 2;
        var packet = new byte[4 + payloadLen];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.CommandList;
        packet[idx++] = (byte)payloadLen;

        foreach (var (opcode, paramCount) in commands)
        {
            packet[idx++] = (byte)opcode;
            packet[idx++] = paramCount;
        }

        packet[idx++] = Etx;
        WritePacket(packet);
    }

    private void SendConfig()
    {
        // 49 bytes config
        var packet = new byte[4 + 49];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.Config;
        packet[idx++] = 49;

        // speed_bps (4 bytes LE)
        var speed = (uint)_speed;
        for (int i = 0; i < 4; i++) packet[idx++] = (byte)(speed >> (i * 8));

        // cnf1, cnf2, cnf3
        packet[idx++] = 0x00;
        packet[idx++] = 0x90;
        packet[idx++] = 0x02;

        // mode
        packet[idx++] = (byte)_mode;

        // flags
        byte flags = 0;
        if (_playbackEngine?.CaptureActive == true) flags |= 0x10;
        packet[idx++] = flags;

        // 6 filters x 5 bytes = 30 bytes (all zeros = disabled)
        for (int i = 0; i < 30; i++) packet[idx++] = 0;

        // 2 masks x 5 bytes = 10 bytes (all zeros)
        for (int i = 0; i < 10; i++) packet[idx++] = 0;

        packet[idx++] = Etx;
        WritePacket(packet);
    }

    private void SendRegisters()
    {
        // 15 bytes raw registers
        var packet = new byte[4 + 15];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)ResponseCode.Registers;
        packet[idx++] = 15;

        // cnf1, cnf2, cnf3
        packet[idx++] = 0x00;
        packet[idx++] = 0x90;
        packet[idx++] = 0x02;

        // canstat (mode in bits 7:5, normal mode = 0)
        packet[idx++] = 0x00;

        // canctrl
        packet[idx++] = 0x00;

        // eflg, canintf
        packet[idx++] = 0x00;
        packet[idx++] = 0x00;

        // tec, rec
        packet[idx++] = 0x00;
        packet[idx++] = 0x00;

        // txb0ctrl, txb1ctrl, txb2ctrl
        packet[idx++] = 0x00;
        packet[idx++] = 0x00;
        packet[idx++] = 0x00;

        // rxb0ctrl, rxb1ctrl
        packet[idx++] = 0x00;
        packet[idx++] = 0x00;

        // mismatch_flags
        packet[idx++] = 0x00;

        packet[idx++] = Etx;
        WritePacket(packet);
    }

    private void WritePacket(byte[] packet)
    {
        lock (_writeLock)
        {
            try
            {
                _port.Write(packet, 0, packet.Length);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Write error: {ex.Message}");
            }
        }
    }
}
