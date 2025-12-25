namespace CANDelta.Core.Protocol;

/// <summary>
/// Represents a single CAN bus frame with timestamp.
/// </summary>
public readonly record struct CanFrame
{
    /// <summary>
    /// CAN identifier (11-bit standard or 29-bit extended).
    /// </summary>
    public uint Id { get; init; }

    /// <summary>
    /// Frame data (0-8 bytes).
    /// </summary>
    public byte[] Data { get; init; }

    /// <summary>
    /// Data length code (0-8).
    /// </summary>
    public byte Dlc { get; init; }

    /// <summary>
    /// True if this is an extended (29-bit) ID frame.
    /// </summary>
    public bool IsExtended { get; init; }

    /// <summary>
    /// True if this is a Remote Transmission Request frame.
    /// </summary>
    public bool IsRtr { get; init; }

    /// <summary>
    /// Capture timestamp in microseconds since device boot.
    /// </summary>
    public ulong TimestampUs { get; init; }

    /// <summary>
    /// Creates a signature for this frame (ID + data pattern) for delta comparison.
    /// </summary>
    public string GetSignature()
    {
        var dataHex = Data != null ? Convert.ToHexString(Data.AsSpan(0, Math.Min(Dlc, Data.Length))) : "";
        return $"{Id:X8}:{IsExtended}:{dataHex}";
    }

    /// <summary>
    /// Creates a signature based only on the CAN ID (ignores data).
    /// </summary>
    public string GetIdSignature()
    {
        return $"{Id:X8}:{IsExtended}";
    }

    public override string ToString()
    {
        var idStr = IsExtended ? $"{Id:X8}" : $"{Id:X3}";
        var dataStr = Data != null ? Convert.ToHexString(Data.AsSpan(0, Math.Min(Dlc, Data.Length))) : "";
        return $"[{TimestampUs,12}] {idStr} [{Dlc}] {dataStr}";
    }
}
