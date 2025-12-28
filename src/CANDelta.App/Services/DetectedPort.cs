namespace CANDelta.App.Services;

/// <summary>
/// Represents a detected serial port with optional USB device information.
/// </summary>
public sealed class DetectedPort
{
    /// <summary>
    /// CANDelta USB identifiers.
    /// VID 0x2E8A = Raspberry Pi Foundation
    /// PID 0xCA1D = CANDelta custom PID
    /// </summary>
    public const string CanDeltaVidPid = "VID_2E8A&PID_CA1D";

    /// <summary>
    /// The system port name (e.g., "COM7" on Windows, "/dev/ttyACM0" on Linux).
    /// </summary>
    public string PortName { get; }

    /// <summary>
    /// User-friendly display name (e.g., "CANDelta (COM7)" or just "COM7").
    /// </summary>
    public string DisplayName { get; }

    /// <summary>
    /// Whether this port is a CANDelta device (based on VID/PID).
    /// </summary>
    public bool IsCanDelta { get; }

    /// <summary>
    /// Whether the CANDelta driver needs to be installed.
    /// True if device is CANDelta but Windows doesn't show the proper name.
    /// </summary>
    public bool NeedsDriverInstall { get; }

    public DetectedPort(string portName, bool isCanDelta = false, bool needsDriverInstall = false)
    {
        PortName = portName;
        IsCanDelta = isCanDelta;
        NeedsDriverInstall = needsDriverInstall;
        DisplayName = IsCanDelta ? $"CANDelta ({portName})" : portName;
    }

    public override string ToString() => DisplayName;

    public override bool Equals(object? obj) =>
        obj is DetectedPort other && PortName == other.PortName;

    public override int GetHashCode() => PortName.GetHashCode();
}
