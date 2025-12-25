using CANDelta.Core.Protocol;

namespace CANDelta.Core.Models;

/// <summary>
/// Represents the result of a delta analysis between control and test behaviours.
/// </summary>
public class DeltaResult
{
    /// <summary>
    /// When the analysis was performed.
    /// </summary>
    public DateTime AnalyzedAt { get; init; } = DateTime.UtcNow;

    /// <summary>
    /// The control behaviour used as baseline.
    /// </summary>
    public required Behaviour ControlBehaviour { get; init; }

    /// <summary>
    /// The test behaviour being compared.
    /// </summary>
    public required Behaviour TestBehaviour { get; init; }

    /// <summary>
    /// CAN IDs that appear only in the test behaviour (not in control).
    /// </summary>
    public List<DeltaFrame> UniqueToTest { get; init; } = new();

    /// <summary>
    /// CAN IDs that appear only in the control behaviour (not in test).
    /// </summary>
    public List<DeltaFrame> UniqueToControl { get; init; } = new();

    /// <summary>
    /// CAN IDs that appear in both but with different data patterns.
    /// </summary>
    public List<DeltaFrame> ModifiedFrames { get; init; } = new();

    /// <summary>
    /// Noise threshold used for the analysis (minimum occurrence percentage).
    /// </summary>
    public double NoiseThreshold { get; init; }

    /// <summary>
    /// Total number of unique CAN IDs in the test behaviour.
    /// </summary>
    public int TotalTestIds => UniqueToTest.Count + ModifiedFrames.Count;

    /// <summary>
    /// Summary of the delta analysis.
    /// </summary>
    public string Summary => $"Found {UniqueToTest.Count} unique test IDs, " +
                             $"{UniqueToControl.Count} unique control IDs, " +
                             $"{ModifiedFrames.Count} modified IDs";
}

/// <summary>
/// Represents a CAN frame pattern found in delta analysis.
/// </summary>
public class DeltaFrame
{
    /// <summary>
    /// The CAN ID.
    /// </summary>
    public uint Id { get; init; }

    /// <summary>
    /// Whether this is an extended ID.
    /// </summary>
    public bool IsExtended { get; init; }

    /// <summary>
    /// Number of times this frame appeared in the source traces.
    /// </summary>
    public int OccurrenceCount { get; init; }

    /// <summary>
    /// Percentage of traces in which this frame appeared.
    /// </summary>
    public double OccurrencePercentage { get; init; }

    /// <summary>
    /// Example frames matching this pattern.
    /// </summary>
    public List<CanFrame> Examples { get; init; } = new();

    /// <summary>
    /// Distinct data patterns observed for this ID.
    /// </summary>
    public List<byte[]> DataPatterns { get; init; } = new();

    /// <summary>
    /// Confidence score for this frame being part of the target feature (0-1).
    /// </summary>
    public double Confidence { get; set; }
}
