namespace CANDelta.Core.Protocol;

/// <summary>
/// Command opcodes sent from host to device.
/// NOTE: 0x02 and 0x03 are reserved for STX/ETX framing.
/// </summary>
public enum CommandOpcode : byte
{
    Ping = 0x01,
    GetVersion = 0x04,
    GetStatus = 0x05,
    Debug = 0x06,
    GetPerfStats = 0x07,
    GetDeviceId = 0x08,
    GetErrorCounters = 0x09,
    ListCommands = 0x0A,
    GetConfig = 0x0B,
    GetRegisters = 0x0C,
    StartCapture = 0x10,
    StopCapture = 0x11,
    SetSpeed = 0x20,
    SetFilter = 0x21,
    ClearFilters = 0x22,
    SetMode = 0x23,
    SetTiming = 0x24,
    SetMask = 0x25,
    SetOneshot = 0x26,
    ResetCan = 0x27,
    TransmitFrame = 0x30,
}

/// <summary>
/// Response codes received from device.
/// </summary>
public enum ResponseCode : byte
{
    Ack = 0x80,
    Nak = 0x81,
    Version = 0x82,
    Status = 0x83,
    CanFrame = 0x84,
    Debug = 0x85,
    PerfStats = 0x86,
    DeviceId = 0x87,
    ErrorCounters = 0x88,
    CommandList = 0x89,
    Config = 0x8A,
    Registers = 0x8B,
}

/// <summary>
/// CAN bus speeds supported by the device.
/// </summary>
public enum CanSpeed : uint
{
    Speed125Kbps = 125000,
    Speed250Kbps = 250000,
    Speed500Kbps = 500000,
    Speed1Mbps = 1000000,
}

/// <summary>
/// MCP2515 operating modes.
/// </summary>
public enum CanMode : byte
{
    Normal = 0,
    Sleep = 1,
    Loopback = 2,
    ListenOnly = 3,
    Config = 4,
}

/// <summary>
/// CAN error states per CAN 2.0B specification.
/// </summary>
public enum CanErrorState : byte
{
    ErrorActive = 0,
    ErrorWarning = 1,
    ErrorPassive = 2,
    BusOff = 3,
}

/// <summary>
/// Device status information.
/// </summary>
public record DeviceStatus
{
    public byte ProtocolVersion { get; init; }
    public CanMode Mode { get; init; }
    public CanSpeed Speed { get; init; }
    public bool CaptureActive { get; init; }
    public byte ErrorFlags { get; init; }
    public uint RxFrameCount { get; init; }
    public uint TxFrameCount { get; init; }
}

/// <summary>
/// Device version information.
/// </summary>
public record DeviceVersion
{
    public byte ProtocolVersion { get; init; }
    public byte Major { get; init; }
    public byte Minor { get; init; }
    public byte Patch { get; init; }

    public override string ToString() => $"v{Major}.{Minor}.{Patch} (protocol {ProtocolVersion})";
}

/// <summary>
/// Performance statistics from the device.
/// </summary>
public record PerfStats
{
    public uint FramesPerSecond { get; init; }
    public uint PeakFps { get; init; }
    public uint DroppedFrames { get; init; }
    public byte BufferUtilization { get; init; }
}

/// <summary>
/// CAN error counters.
/// </summary>
public record ErrorCounters
{
    public byte Tec { get; init; }
    public byte Rec { get; init; }
    public CanErrorState State { get; init; }
}
