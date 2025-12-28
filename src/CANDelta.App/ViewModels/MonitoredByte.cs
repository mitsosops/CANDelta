using Avalonia.Media;
using CommunityToolkit.Mvvm.ComponentModel;

namespace CANDelta.App.ViewModels;

/// <summary>
/// Represents a single byte in a CAN frame with value tracking and animated color transitions.
/// Tracks min/max observed values to calculate change intensity.
/// </summary>
public partial class MonitoredByte : ObservableObject
{
    // Idle colors: neutral gray for light/dark mode compatibility
    private static readonly Color IdleForeground = Color.Parse("#888888");
    private static readonly Color IdleBackground = Colors.Transparent;

    private byte _value;
    private byte _previousValue;
    private byte _minObserved = 255;
    private byte _maxObserved = 0;
    private int _sampleCount;

    [ObservableProperty]
    private double _intensity; // 0.0=idle, 1.0=max alert

    [ObservableProperty]
    private SolidColorBrush _foreground = new(IdleForeground);

    [ObservableProperty]
    private SolidColorBrush _background = new(IdleBackground);

    [ObservableProperty]
    private string _displayText = "--";

    /// <summary>
    /// Current byte value.
    /// </summary>
    public byte Value => _value;

    /// <summary>
    /// Whether this byte is currently animating (intensity > 0).
    /// </summary>
    public bool IsAnimating => Intensity > 0.001;

    /// <summary>
    /// Updates the byte value and triggers color animation.
    /// </summary>
    public void UpdateValue(byte newValue, Color themeColor)
    {
        _previousValue = _value;
        _value = newValue;
        _sampleCount++;

        DisplayText = newValue.ToString("X2");

        // Update min/max tracking
        if (newValue < _minObserved) _minObserved = newValue;
        if (newValue > _maxObserved) _maxObserved = newValue;

        // Calculate intensity based on change magnitude
        Intensity = CalculateIntensity(newValue);

        // Apply colors immediately
        ApplyColors(themeColor);
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
        Intensity = 0;
        DisplayText = "--";
        Foreground.Color = IdleForeground;
        Background.Color = IdleBackground;
        OnPropertyChanged(nameof(Foreground));
        OnPropertyChanged(nameof(Background));
    }
}
