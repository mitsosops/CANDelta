using Avalonia;
using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;

namespace CANDelta.App.ViewModels;

/// <summary>
/// Represents a single byte in a CAN frame with value tracking and animated color transitions.
/// Tracks min/max observed values to calculate change intensity.
/// Also maintains a history buffer for sparkline visualization.
/// </summary>
public partial class MonitoredByte : ObservableObject
{
    // Idle colors: neutral gray for light/dark mode compatibility
    private static readonly Color IdleForeground = Color.Parse("#888888");
    private static readonly Color IdleBackground = Colors.Transparent;

    // Pre-computed hex strings to avoid ToString allocations (256 possible byte values)
    private static readonly string[] HexStrings = CreateHexStringCache();

    private static string[] CreateHexStringCache()
    {
        var cache = new string[256];
        for (int i = 0; i < 256; i++)
        {
            cache[i] = i.ToString("X2");
        }
        return cache;
    }

    // History settings
    private const int HistoryDurationMs = 5000; // 5 seconds
    private const int HistoryCapacity = 2000;   // Fixed capacity (supports up to 400Hz for 5 seconds)

    private byte _value;
    private byte _previousValue;
    private byte _minObserved = 255;
    private byte _maxObserved = 0;
    private int _sampleCount;

    // Circular buffer for history: avoids O(n) RemoveAt(0) operations
    private readonly (long timestamp, byte value)[] _historyBuffer = new (long, byte)[HistoryCapacity];
    private int _historyHead;  // Index of oldest item
    private int _historyCount; // Number of items in buffer

    // Dirty flag for graph updates - only rebuild when data changed
    private bool _graphDirty;

    // Reusable Points collection to avoid allocations
    private Points _graphPointsInternal = new();

    [ObservableProperty]
    private double _intensity; // 0.0=idle, 1.0=max alert

    [ObservableProperty]
    private SolidColorBrush _foreground = new(IdleForeground);

    [ObservableProperty]
    private SolidColorBrush _background = new(IdleBackground);

    [ObservableProperty]
    private string _displayText = "--";

    [ObservableProperty]
    private Points _graphPoints = new();

    /// <summary>
    /// Current byte value.
    /// </summary>
    public byte Value => _value;

    /// <summary>
    /// Minimum observed value (for graph scaling).
    /// </summary>
    public byte MinObserved => _minObserved;

    /// <summary>
    /// Maximum observed value (for graph scaling).
    /// </summary>
    public byte MaxObserved => _maxObserved;

    /// <summary>
    /// Whether this byte is currently animating (intensity > 0).
    /// </summary>
    public bool IsAnimating => Intensity > 0.001;

    /// <summary>
    /// Whether this byte has enough data for a meaningful graph.
    /// </summary>
    public bool HasGraphData => _historyCount >= 2;

    /// <summary>
    /// Updates the byte value and triggers color animation.
    /// </summary>
    public void UpdateValue(byte newValue, Color themeColor)
    {
        _previousValue = _value;
        _value = newValue;
        _sampleCount++;

        DisplayText = HexStrings[newValue]; // Use cached string to avoid allocation

        // Update min/max tracking
        if (newValue < _minObserved) _minObserved = newValue;
        if (newValue > _maxObserved) _maxObserved = newValue;

        // Add to circular buffer
        long now = Environment.TickCount64;
        AddToHistory(now, newValue);

        // Prune old points (older than 5 seconds) - O(1) per removal with circular buffer
        long cutoff = now - HistoryDurationMs;
        while (_historyCount > 0 && _historyBuffer[_historyHead].timestamp < cutoff)
        {
            _historyHead = (_historyHead + 1) % HistoryCapacity;
            _historyCount--;
        }

        // Mark graph as needing update
        _graphDirty = true;

        // Calculate intensity based on change magnitude
        Intensity = CalculateIntensity(newValue);

        // Apply colors immediately
        ApplyColors(themeColor);
    }

    /// <summary>
    /// Adds a point to the circular history buffer. O(1) operation.
    /// </summary>
    private void AddToHistory(long timestamp, byte value)
    {
        int tail = (_historyHead + _historyCount) % HistoryCapacity;

        if (_historyCount < HistoryCapacity)
        {
            // Buffer not full, just append
            _historyBuffer[tail] = (timestamp, value);
            _historyCount++;
        }
        else
        {
            // Buffer full, overwrite oldest (head moves forward)
            _historyBuffer[tail] = (timestamp, value);
            _historyHead = (_historyHead + 1) % HistoryCapacity;
            // _historyCount stays at HistoryCapacity
        }
    }

    /// <summary>
    /// Updates the graph points for rendering. Call periodically from UI timer.
    /// Only rebuilds if data has changed since last call.
    /// </summary>
    /// <param name="graphWidth">Width of the graph area in pixels.</param>
    /// <param name="graphHeight">Height of the graph area in pixels.</param>
    public void UpdateGraphPoints(double graphWidth, double graphHeight)
    {
        // Skip update if nothing changed
        if (!_graphDirty) return;
        _graphDirty = false;

        if (_historyCount < 2 || _minObserved == _maxObserved)
        {
            if (_graphPointsInternal.Count > 0)
            {
                _graphPointsInternal.Clear();
                GraphPoints = _graphPointsInternal;
                OnPropertyChanged(nameof(GraphPoints));
            }
            return;
        }

        // Reuse existing collection - clear instead of allocating new
        _graphPointsInternal.Clear();

        long now = Environment.TickCount64;
        long windowStart = now - HistoryDurationMs;
        int range = _maxObserved - _minObserved;

        // Iterate circular buffer
        for (int i = 0; i < _historyCount; i++)
        {
            int idx = (_historyHead + i) % HistoryCapacity;
            var (timestamp, value) = _historyBuffer[idx];

            // X: map timestamp to 0..graphWidth (oldest on left, newest on right)
            double x = (timestamp - windowStart) / (double)HistoryDurationMs * graphWidth;

            // Y: map value to graphHeight..0 (min at bottom, max at top)
            double normalizedY = (value - _minObserved) / (double)range;
            double y = graphHeight - (normalizedY * graphHeight);

            // Clamp x to visible area
            if (x >= 0 && x <= graphWidth)
            {
                _graphPointsInternal.Add(new Point(x, Math.Clamp(y, 0, graphHeight)));
            }
        }

        // Trigger binding update (GraphPoints already references _graphPointsInternal)
        GraphPoints = _graphPointsInternal;
        OnPropertyChanged(nameof(GraphPoints));
    }

    /// <summary>
    /// Calculates intensity (0-1) based on value change relative to observed range.
    /// </summary>
    private double CalculateIntensity(byte newValue)
    {
        // First two samples always get max intensity to establish the baseline
        if (_sampleCount <= 2)
        {
            return 1.0;
        }

        // If value didn't change, no flash
        if (newValue == _previousValue)
        {
            return Intensity; // Keep current intensity (will fade)
        }

        // Calculate delta relative to observed range
        int range = _maxObserved - _minObserved;
        if (range == 0)
        {
            // All values same so far, any change is significant
            return 1.0;
        }

        int delta = Math.Abs(newValue - _previousValue);
        double normalized = (double)delta / range;

        // Clamp to 0-1 and apply minimum intensity for any change
        return Math.Clamp(normalized, 0.15, 1.0);
    }

    /// <summary>
    /// Applies fade-back animation. Called by timer.
    /// </summary>
    /// <param name="fadeAmount">Amount to reduce intensity (typically 0.02-0.05 per tick).</param>
    /// <param name="themeColor">Current theme color for interpolation.</param>
    public void ApplyFade(double fadeAmount, Color themeColor)
    {
        if (Intensity <= 0) return;

        Intensity = Math.Max(0, Intensity - fadeAmount);
        ApplyColors(themeColor);
    }

    /// <summary>
    /// Applies colors based on current intensity and theme.
    /// </summary>
    private void ApplyColors(Color themeColor)
    {
        // Lerp between idle and theme colors based on intensity
        var fgColor = LerpColor(IdleForeground, themeColor, Intensity);
        var bgColor = LerpColor(IdleBackground, Color.FromArgb((byte)(themeColor.A * 0.3), themeColor.R, themeColor.G, themeColor.B), Intensity);

        // Mutate existing brushes to avoid allocations
        Foreground.Color = fgColor;
        Background.Color = bgColor;

        // Force property change notification for UI update
        OnPropertyChanged(nameof(Foreground));
        OnPropertyChanged(nameof(Background));
    }

    /// <summary>
    /// Linear interpolation between two colors.
    /// </summary>
    private static Color LerpColor(Color from, Color to, double t)
    {
        t = Math.Clamp(t, 0, 1);
        return Color.FromArgb(
            (byte)(from.A + (to.A - from.A) * t),
            (byte)(from.R + (to.R - from.R) * t),
            (byte)(from.G + (to.G - from.G) * t),
            (byte)(from.B + (to.B - from.B) * t)
        );
    }

    /// <summary>
    /// Resets the byte to initial state.
    /// </summary>
    public void Reset()
    {
        _value = 0;
        _previousValue = 0;
        _minObserved = 255;
        _maxObserved = 0;
        _sampleCount = 0;

        // Reset circular buffer (just reset indices, no need to clear array)
        _historyHead = 0;
        _historyCount = 0;
        _graphDirty = false;

        Intensity = 0;
        DisplayText = "--";

        // Clear and reuse existing Points collection
        _graphPointsInternal.Clear();
        GraphPoints = _graphPointsInternal;

        Foreground.Color = IdleForeground;
        Background.Color = IdleBackground;
        OnPropertyChanged(nameof(Foreground));
        OnPropertyChanged(nameof(Background));
        OnPropertyChanged(nameof(GraphPoints));
    }
}
