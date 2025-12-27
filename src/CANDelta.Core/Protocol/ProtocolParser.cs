namespace CANDelta.Core.Protocol;

/// <summary>
/// Parses and serializes the wire protocol between host and device.
/// </summary>
public class ProtocolParser
{
    private const byte Stx = 0x02;
    private const byte Etx = 0x03;

    private readonly List<byte> _buffer = new();
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
                    _buffer.Clear();
                    _state = ParseState.WaitingForOpcode;
                }
                break;

            case ParseState.WaitingForOpcode:
                _buffer.Add(b);
                _state = ParseState.WaitingForLength;
                break;

            case ParseState.WaitingForLength:
                _buffer.Add(b);
                _payloadLength = b;
                _payloadRead = 0;
                _state = _payloadLength > 0 ? ParseState.ReadingPayload : ParseState.WaitingForEtx;
                break;

            case ParseState.ReadingPayload:
                _buffer.Add(b);
                _payloadRead++;
                if (_payloadRead >= _payloadLength)
                {
                    _state = ParseState.WaitingForEtx;
                }
                break;

            case ParseState.WaitingForEtx:
                if (b == Etx)
                {
                    ProcessPacket(_buffer.ToArray());
                }
                // Reset state regardless (handles framing errors by resyncing)
                _buffer.Clear();
                _state = ParseState.WaitingForStx;
                break;
        }
    }

    private void ProcessPacket(byte[] data)
    {
        if (data.Length < 2) return;

        var code = (ResponseCode)data[0];
        var len = data[1];
        var payload = data.Length > 2 ? data[2..Math.Min(2 + len, data.Length)] : Array.Empty<byte>();

        // CAN frame responses get special handling
        if (code == ResponseCode.CanFrame)
        {
            var frame = ParseCanFrame(payload);
            if (frame.HasValue)
            {
                FrameReceived?.Invoke(frame.Value);
            }
            return;
        }

        // All other responses
        ResponseReceived?.Invoke(code, payload);
    }

    private static CanFrame? ParseCanFrame(byte[] data)
    {
        if (data.Length < 14) return null;

        int idx = 0;

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

        // Data
        byte[] frameData = new byte[dlc];
        for (int i = 0; i < dlc && idx < data.Length; i++)
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
