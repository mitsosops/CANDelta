using Avalonia.Media;
using CANDelta.Core.Protocol;
using CommunityToolkit.Mvvm.ComponentModel;

namespace CANDelta.App.ViewModels;

/// <summary>
/// Represents a monitored CAN ID with its 8 data bytes and animation state.
/// </summary>
public partial class MonitoredCanId : ObservableObject, IComparable<MonitoredCanId>
{
    /// <summary>
    /// The CAN ID being monitored.
    /// </summary>
    public uint Id { get; }

    /// <summary>
    /// Whether this is an extended (29-bit) ID.
    /// </summary>
    public bool IsExtended { get; }

    /// <summary>
    /// Display string for the CAN ID (hex format).
    /// Standard IDs: 3 digits, Extended IDs: 8 digits.
    /// </summary>
    [ObservableProperty]
    private string _idDisplay;

    /// <summary>
    /// The 8 data bytes with individual tracking and animation.
    /// </summary>
    public MonitoredByte[] Bytes { get; } = new MonitoredByte[8];

    /// <summary>
    /// Total number of frames received for this ID.
    /// </summary>
    [ObservableProperty]
    private int _frameCount;

    /// <summary>
    /// Last DLC received (for proper byte display).
    /// </summary>
    public byte LastDlc { get; private set; }

    public MonitoredCanId(uint id, bool isExtended)
    {
        Id = id;
        IsExtended = isExtended;
        _idDisplay = isExtended ? id.ToString("X8") : id.ToString("X3");

        // Initialize all 8 bytes
        for (int i = 0; i < 8; i++)
        {
            Bytes[i] = new MonitoredByte();
        }
    }

    /// <summary>
    /// Updates this entry from a received CAN frame.
    /// </summary>
    public void UpdateFromFrame(CanFrame frame, Color themeColor)
    {
        FrameCount++;
        LastDlc = frame.Dlc;

        // Update each byte up to DLC
        for (int i = 0; i < frame.Dlc && i < 8; i++)
        {
            byte value = i < frame.Data.Length ? frame.Data[i] : (byte)0;
            Bytes[i].UpdateValue(value, themeColor);
        }

        // Mark unused bytes as empty (if DLC changed)
        for (int i = frame.Dlc; i < 8; i++)
        {
            if (Bytes[i].DisplayText != "--")
            {
                Bytes[i].Reset();
            }
        }
    }

    /// <summary>
    /// Applies fade animation to all bytes.
    /// </summary>
    /// <param name="fadeAmount">Amount to reduce intensity per tick.</param>
    /// <param name="themeColor">Current theme color.</param>
    public void ApplyFade(double fadeAmount, Color themeColor)
    {
        for (int i = 0; i < 8; i++)
        {
            Bytes[i].ApplyFade(fadeAmount, themeColor);
        }
    }

    /// <summary>
    /// Returns true if any byte is still animating.
    /// </summary>
    public bool HasActiveAnimation()
    {
        for (int i = 0; i < 8; i++)
        {
            if (Bytes[i].IsAnimating) return true;
        }
        return false;
    }

    /// <summary>
    /// Resets all bytes to initial state.
    /// </summary>
    public void Reset()
    {
        FrameCount = 0;
        LastDlc = 0;
        for (int i = 0; i < 8; i++)
        {
            Bytes[i].Reset();
        }
    }

    /// <summary>
    /// Updates graph points for all bytes.
    /// </summary>
    /// <param name="graphWidth">Width of each graph in pixels.</param>
    /// <param name="graphHeight">Height of each graph in pixels.</param>
    public void UpdateGraphPoints(double graphWidth, double graphHeight)
    {
        for (int i = 0; i < LastDlc && i < 8; i++)
        {
            Bytes[i].UpdateGraphPoints(graphWidth, graphHeight);
        }
    }

    /// <summary>
    /// Compare by CAN ID for sorted insertion.
    /// </summary>
    public int CompareTo(MonitoredCanId? other)
    {
        if (other is null) return 1;
        return Id.CompareTo(other.Id);
    }
}
