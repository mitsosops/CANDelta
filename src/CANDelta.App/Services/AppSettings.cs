using System.Text.Json;

namespace CANDelta.App.Services;

/// <summary>
/// Persists application settings across sessions.
/// </summary>
public static class AppSettings
{
    private static readonly string SettingsPath = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "CANDelta",
        "settings.json"
    );

    private static Settings? _cached;

    /// <summary>
    /// Gets or sets whether the driver install prompt has been dismissed.
    /// </summary>
    public static bool DriverPromptDismissed
    {
        get => Load().DriverPromptDismissed;
        set
        {
            var settings = Load();
            settings.DriverPromptDismissed = value;
            Save(settings);
        }
    }

    /// <summary>
    /// Gets or sets the selected color theme name.
    /// </summary>
    public static string? SelectedThemeName
    {
        get => Load().SelectedThemeName;
        set
        {
            var settings = Load();
            settings.SelectedThemeName = value;
            Save(settings);
        }
    }

    private static Settings Load()
    {
        if (_cached != null) return _cached;

        try
        {
            if (File.Exists(SettingsPath))
            {
                var json = File.ReadAllText(SettingsPath);
                _cached = JsonSerializer.Deserialize<Settings>(json) ?? new Settings();
                return _cached;
            }
        }
        catch
        {
            // Ignore load errors
        }

        _cached = new Settings();
        return _cached;
    }

    private static void Save(Settings settings)
    {
        try
        {
            var dir = Path.GetDirectoryName(SettingsPath);
            if (dir != null && !Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }

            var json = JsonSerializer.Serialize(settings, new JsonSerializerOptions { WriteIndented = true });
            File.WriteAllText(SettingsPath, json);
            _cached = settings;
        }
        catch
        {
            // Ignore save errors
        }
    }

    private class Settings
    {
        public bool DriverPromptDismissed { get; set; }
        public string? SelectedThemeName { get; set; }
    }
}
