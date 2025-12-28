using Avalonia.Media;

namespace CANDelta.App.ViewModels;

/// <summary>
/// Defines a color theme for the CAN monitor byte highlighting.
/// </summary>
public class ColorTheme
{
    /// <summary>
    /// Display name for the theme.
    /// </summary>
    public string Name { get; }

    /// <summary>
    /// The alert color used for value changes.
    /// </summary>
    public Color AlertColor { get; }

    public ColorTheme(string name, Color alertColor)
    {
        Name = name;
        AlertColor = alertColor;
    }

    /// <summary>
    /// Available preset themes.
    /// </summary>
    public static ColorTheme[] AvailableThemes { get; } = new[]
    {
        new ColorTheme("Cyan", Color.Parse("#00DDDD")),
        new ColorTheme("Orange", Color.Parse("#FF8800")),
        new ColorTheme("Lime", Color.Parse("#88FF00")),
        new ColorTheme("Red", Color.Parse("#FF4444")),
        new ColorTheme("Purple", Color.Parse("#BB66FF")),
        new ColorTheme("Yellow", Color.Parse("#FFDD00")),
        new ColorTheme("Pink", Color.Parse("#FF66AA")),
        new ColorTheme("Blue", Color.Parse("#4488FF")),
    };

    public override string ToString() => Name;
}
