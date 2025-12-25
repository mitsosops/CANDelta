namespace CANDelta.Core.Models;

/// <summary>
/// Represents a behaviour being analyzed (control or test).
/// A behaviour contains multiple traces to help filter out noise.
/// </summary>
public class Behaviour
{
    /// <summary>
    /// Unique identifier for this behaviour.
    /// </summary>
    public Guid Id { get; init; } = Guid.NewGuid();

    /// <summary>
    /// User-defined name for this behaviour.
    /// </summary>
    public string Name { get; set; } = string.Empty;

    /// <summary>
    /// Optional description of what this behaviour represents.
    /// </summary>
    public string Description { get; set; } = string.Empty;

    /// <summary>
    /// When this behaviour was created.
    /// </summary>
    public DateTime CreatedAt { get; init; } = DateTime.UtcNow;

    /// <summary>
    /// All traces captured for this behaviour.
    /// </summary>
    public List<Trace> Traces { get; init; } = new();

    /// <summary>
    /// Whether this is a control (baseline) behaviour.
    /// </summary>
    public bool IsControl { get; set; }
}
