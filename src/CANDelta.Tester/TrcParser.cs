using System.Globalization;

namespace CANDelta.Tester;

/// <summary>
/// Represents a single CAN frame from a .trc file.
/// </summary>
public record TrcFrame(double TimestampSeconds, uint CanId, byte[] Data);

/// <summary>
/// Parses CANHacker .trc trace files.
/// </summary>
public static class TrcParser
{
    /// <summary>
    /// Parse a .trc file and return a list of CAN frames.
    /// </summary>
    /// <param name="filePath">Path to the .trc file.</param>
    /// <returns>List of parsed frames in chronological order.</returns>
    public static List<TrcFrame> Parse(string filePath)
    {
        var frames = new List<TrcFrame>();
        var lines = File.ReadAllLines(filePath);

        foreach (var line in lines)
        {
            var trimmed = line.Trim();

            // Skip empty lines and comments/headers
            if (string.IsNullOrEmpty(trimmed) ||
                trimmed.StartsWith(";") ||
                trimmed.StartsWith("Time") ||
                trimmed.StartsWith("---"))
            {
                continue;
            }

            var frame = ParseLine(trimmed);
            if (frame != null)
            {
                frames.Add(frame);
            }
        }

        return frames;
    }

    private static TrcFrame? ParseLine(string line)
    {
        // Format: "Time       ID   DLC  Data"
        // Example: "07,399     256  8    12 34 56 78 9A BC DE F0"
        // Time uses comma as decimal separator (European format)

        var parts = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 3)
        {
            return null;
        }

        try
        {
            // Parse timestamp (comma as decimal separator)
            var timeStr = parts[0].Replace(',', '.');
            if (!double.TryParse(timeStr, NumberStyles.Float, CultureInfo.InvariantCulture, out var timestamp))
            {
                return null;
            }

            // Parse CAN ID (decimal)
            if (!uint.TryParse(parts[1], out var canId))
            {
                // Try hex format (some files use 0x prefix or just hex)
                if (parts[1].StartsWith("0x", StringComparison.OrdinalIgnoreCase))
                {
                    if (!uint.TryParse(parts[1][2..], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out canId))
                    {
                        return null;
                    }
                }
                else if (!uint.TryParse(parts[1], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out canId))
                {
                    return null;
                }
            }

            // Parse DLC
            if (!int.TryParse(parts[2], out var dlc))
            {
                return null;
            }
            dlc = Math.Min(dlc, 8);

            // Parse data bytes (hex)
            var data = new byte[dlc];
            for (int i = 0; i < dlc && i + 3 < parts.Length; i++)
            {
                if (byte.TryParse(parts[i + 3], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var b))
                {
                    data[i] = b;
                }
            }

            return new TrcFrame(timestamp, canId, data);
        }
        catch
        {
            return null;
        }
    }
}
