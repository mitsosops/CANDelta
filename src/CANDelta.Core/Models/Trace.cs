using CANDelta.Core.Protocol;

namespace CANDelta.Core.Models;

/// <summary>
/// Represents a single CAN trace capture session.
/// </summary>
public class Trace
{
    /// <summary>
    /// Unique identifier for this trace.
    /// </summary>
    public Guid Id { get; init; } = Guid.NewGuid();

    /// <summary>
    /// When this trace was started.
    /// </summary>
    public DateTime StartTime { get; set; } = DateTime.UtcNow;

    /// <summary>
    /// When this trace was stopped.
    /// </summary>
    public DateTime? EndTime { get; set; }

    /// <summary>
    /// Duration of the trace capture.
    /// </summary>
    public TimeSpan Duration => EndTime.HasValue ? EndTime.Value - StartTime : TimeSpan.Zero;

    /// <summary>
    /// All CAN frames captured in this trace.
    /// </summary>
    public List<CanFrame> Frames { get; init; } = new();

    /// <summary>
    /// Optional user notes about this trace.
    /// </summary>
    public string Notes { get; set; } = string.Empty;

    /// <summary>
    /// CAN bus speed used during capture.
    /// </summary>
    public CanSpeed Speed { get; set; } = CanSpeed.Speed500Kbps;
}
