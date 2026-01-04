using System.Buffers;

namespace CANDelta.Core.Protocol;

/// <summary>
/// Parses and serializes the wire protocol between host and device.
/// </summary>
public class ProtocolParser
{
    private const byte Stx = 0x02;
    private const byte Etx = 0x03;

    // Fixed buffer sized for largest expected packet (Config response = 49 bytes payload + 2 header)
    // See protocol.md for payload sizes
    private const int MaxPacketSize = 64;
    private readonly byte[] _buffer = new byte[MaxPacketSize];
    private int _bufferPos;

    private enum ParseState { WaitingForStx, WaitingForOpcode, WaitingForLength, ReadingPayload, WaitingForEtx }
    private ParseState _state = ParseState.WaitingForStx;
    private int _payloadLength;
    private int _payloadRead;

    /// <summary>
    /// Event raised when a CAN frame is received.
    /// </summary>
    public event Action<CanFrame>? FrameReceived;

    /// <summary>
    /// Event raised when a response is received.
    /// </summary>
    public event Action<ResponseCode, byte[]>? ResponseReceived;

    /// <summary>
    /// Process incoming bytes from the device.
    /// </summary>
    public void ProcessBytes(byte[] data, int offset, int count)
    {
        for (int i = 0; i < count; i++)
        {
            ProcessByte(data[offset + i]);
        }
    }

    private void ProcessByte(byte b)
    {
        switch (_state)
        {
            case ParseState.WaitingForStx:
                if (b == Stx)
                {
                    _bufferPos = 0;
                    _state = ParseState.WaitingForOpcode;
                }
                break;

            case ParseState.WaitingForOpcode:
                if (_bufferPos < MaxPacketSize) _buffer[_bufferPos++] = b;
                _state = ParseState.WaitingForLength;
                break;

            case ParseState.WaitingForLength:
                if (_bufferPos < MaxPacketSize) _buffer[_bufferPos++] = b;
                _payloadLength = b;
                _payloadRead = 0;
                _state = _payloadLength > 0 ? ParseState.ReadingPayload : ParseState.WaitingForEtx;
                break;

            case ParseState.ReadingPayload:
                if (_bufferPos < MaxPacketSize) _buffer[_bufferPos++] = b;
                _payloadRead++;
                if (_payloadRead >= _payloadLength)
                {
                    _state = ParseState.WaitingForEtx;
                }
                break;

            case ParseState.WaitingForEtx:
                if (b == Etx)
                {
                    ProcessPacket(_buffer, _bufferPos);
                }
                // Reset state regardless (handles framing errors by resyncing)
                _bufferPos = 0;
                _state = ParseState.WaitingForStx;
                break;
        }
    }

    private void ProcessPacket(byte[] data, int length)
    {
        if (length < 2) return;

        var code = (ResponseCode)data[0];
        var payloadLen = data[1];

        // CAN frame responses get special handling - hot path, avoid allocations
        if (code == ResponseCode.CanFrame)
        {
            var frame = ParseCanFrame(data, 2, Math.Min(payloadLen, length - 2));
            if (frame.HasValue)
            {
                FrameReceived?.Invoke(frame.Value);
            }
            return;
        }

        // Other responses are less frequent - allocation is acceptable
        var payload = length > 2 ? data[2..Math.Min(2 + payloadLen, length)] : Array.Empty<byte>();
        ResponseReceived?.Invoke(code, payload);
    }

    // Pool for frame data arrays - all CAN frames have 0-8 bytes, rent fixed size 8
    private static readonly ArrayPool<byte> FrameDataPool = ArrayPool<byte>.Create(8, 256);

    private static CanFrame? ParseCanFrame(byte[] data, int offset, int length)
    {
        if (length < 14) return null;

        int idx = offset;
        int end = offset + length;

        // Timestamp (8 bytes LE)
        ulong timestamp = 0;
        for (int i = 0; i < 8; i++)
        {
            timestamp |= (ulong)data[idx++] << (i * 8);
        }

        // ID (4 bytes LE)
        uint id = 0;
        for (int i = 0; i < 4; i++)
        {
            id |= (uint)data[idx++] << (i * 8);
        }

        // Flags
        byte flags = data[idx++];
        bool extended = (flags & 0x01) != 0;
        bool rtr = (flags & 0x02) != 0;

        // DLC
        byte dlc = Math.Min(data[idx++], (byte)8);

        // Data - rent from pool instead of allocating
        byte[] frameData = FrameDataPool.Rent(8);
        for (int i = 0; i < dlc && idx < end; i++)
        {
            frameData[i] = data[idx++];
        }

        return new CanFrame
        {
            Id = id,
            Data = frameData,
            Dlc = dlc,
            IsExtended = extended,
            IsRtr = rtr,
            TimestampUs = timestamp
        };
    }

    /// <summary>
    /// Returns a frame's data array to the pool. Call after processing the frame.
    /// </summary>
    public static void ReturnFrameData(byte[] data)
    {
        if (data != null && data.Length >= 8)
        {
            FrameDataPool.Return(data);
        }
    }

    /// <summary>
    /// Serialize a command to send to the device.
    /// </summary>
    public static byte[] SerializeCommand(CommandOpcode opcode, byte[]? parameters = null)
    {
        parameters ??= Array.Empty<byte>();

        var packet = new byte[4 + parameters.Length];
        int idx = 0;

        packet[idx++] = Stx;
        packet[idx++] = (byte)opcode;
        packet[idx++] = (byte)parameters.Length;

        Array.Copy(parameters, 0, packet, idx, parameters.Length);
        idx += parameters.Length;

        packet[idx++] = Etx;

        return packet;
    }

    /// <summary>
    /// Create a SET_SPEED command.
    /// </summary>
    public static byte[] CreateSetSpeedCommand(CanSpeed speed)
    {
        var speedValue = (uint)speed;
        var parameters = new byte[4];
        parameters[0] = (byte)(speedValue & 0xFF);
        parameters[1] = (byte)((speedValue >> 8) & 0xFF);
        parameters[2] = (byte)((speedValue >> 16) & 0xFF);
        parameters[3] = (byte)((speedValue >> 24) & 0xFF);
        return SerializeCommand(CommandOpcode.SetSpeed, parameters);
    }

    /// <summary>
    /// Create a TRANSMIT_FRAME command.
    /// </summary>
    public static byte[] CreateTransmitCommand(CanFrame frame)
    {
        var parameters = new byte[6 + frame.Dlc];
        int idx = 0;

        // ID (4 bytes LE)
        parameters[idx++] = (byte)(frame.Id & 0xFF);
        parameters[idx++] = (byte)((frame.Id >> 8) & 0xFF);
        parameters[idx++] = (byte)((frame.Id >> 16) & 0xFF);
        parameters[idx++] = (byte)((frame.Id >> 24) & 0xFF);

        // Flags
        parameters[idx++] = (byte)((frame.IsExtended ? 0x01 : 0) | (frame.IsRtr ? 0x02 : 0));

        // DLC
        parameters[idx++] = frame.Dlc;

        // Data
        if (frame.Data != null)
        {
            Array.Copy(frame.Data, 0, parameters, idx, Math.Min(frame.Dlc, frame.Data.Length));
        }

        return SerializeCommand(CommandOpcode.TransmitFrame, parameters);
    }

    /// <summary>
    /// Parse a VERSION response.
    /// </summary>
    public static DeviceVersion? ParseVersionResponse(byte[] data)
    {
        if (data.Length < 4) return null;

        return new DeviceVersion
        {
            ProtocolVersion = data[0],
            Major = data[1],
            Minor = data[2],
            Patch = data[3]
        };
    }

    /// <summary>
    /// Parse a STATUS response.
    /// </summary>
    public static DeviceStatus? ParseStatusResponse(byte[] data)
    {
        if (data.Length < 15) return null;

        int idx = 0;
        var protocolVersion = data[idx++];
        var mode = (CanMode)data[idx++];

        uint speed = 0;
        for (int i = 0; i < 4; i++)
        {
            speed |= (uint)data[idx++] << (i * 8);
        }

        var captureActive = data[idx++] != 0;
        var errorFlags = data[idx++];

        uint rxCount = 0;
        for (int i = 0; i < 4; i++)
        {
            rxCount |= (uint)data[idx++] << (i * 8);
        }

        uint txCount = 0;
        for (int i = 0; i < 4 && idx < data.Length; i++)
        {
            txCount |= (uint)data[idx++] << (i * 8);
        }

        return new DeviceStatus
        {
            ProtocolVersion = protocolVersion,
            Mode = mode,
            Speed = (CanSpeed)speed,
            CaptureActive = captureActive,
            ErrorFlags = errorFlags,
            RxFrameCount = rxCount,
            TxFrameCount = txCount
        };
    }
}
