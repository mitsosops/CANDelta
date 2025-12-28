using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.Versioning;

namespace CANDelta.App.Services;

/// <summary>
/// Handles installation of the CANDelta USB driver (INF file).
/// </summary>
public static class DriverInstaller
{
    private const string InfFileName = "candelta.inf";

    /// <summary>
    /// Installs the CANDelta driver using pnputil.
    /// This will trigger a UAC elevation prompt.
    /// </summary>
    /// <returns>True if installation succeeded, false otherwise.</returns>
    [SupportedOSPlatform("windows")]
    public static async Task<DriverInstallResult> InstallDriverAsync()
    {
        if (!RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
        {
            return new DriverInstallResult(false, "Driver installation is only supported on Windows.");
        }

        string? tempInfPath = null;

        try
        {
            // Extract INF to temp directory
            tempInfPath = await ExtractInfToTempAsync();
            if (tempInfPath == null)
            {
                return new DriverInstallResult(false, "Failed to extract driver file.");
            }

            // Run pnputil with elevation to install the driver
            var psi = new ProcessStartInfo
            {
                FileName = "pnputil.exe",
                Arguments = $"/add-driver \"{tempInfPath}\" /install",
                UseShellExecute = true,
                Verb = "runas", // Request elevation
                CreateNoWindow = false
            };

            using var process = Process.Start(psi);
            if (process == null)
            {
                return new DriverInstallResult(false, "Failed to start driver installer.");
            }

            await process.WaitForExitAsync();

            if (process.ExitCode == 0)
            {
                return new DriverInstallResult(true, "Driver installed successfully. Please reconnect your CANDelta device.");
            }
            else
            {
                return new DriverInstallResult(false, $"Driver installation failed (exit code {process.ExitCode}).");
            }
        }
        catch (System.ComponentModel.Win32Exception ex) when (ex.NativeErrorCode == 1223)
        {
            // ERROR_CANCELLED - User declined UAC prompt
            return new DriverInstallResult(false, "Driver installation was cancelled.");
        }
        catch (Exception ex)
        {
            return new DriverInstallResult(false, $"Driver installation error: {ex.Message}");
        }
        finally
        {
            // Clean up temp file
            if (tempInfPath != null)
            {
                try { File.Delete(tempInfPath); } catch { }
            }
        }
    }

    /// <summary>
    /// Extracts the embedded INF file to a temporary location.
    /// </summary>
    private static async Task<string?> ExtractInfToTempAsync()
    {
        try
        {
            // Get the INF content from the drivers folder relative to the app
            var appDir = AppContext.BaseDirectory;
            var driversDir = Path.Combine(appDir, "drivers");
            var sourceInfPath = Path.Combine(driversDir, InfFileName);

            // If INF exists in drivers folder, use it directly
            if (File.Exists(sourceInfPath))
            {
                var tempPath = Path.Combine(Path.GetTempPath(), $"candelta_{Guid.NewGuid():N}.inf");
                await CopyFileAsync(sourceInfPath, tempPath);
                return tempPath;
            }

            // Try parent directories (for development)
            var searchDir = appDir;
            for (int i = 0; i < 6; i++)
            {
                var candidatePath = Path.Combine(searchDir, "drivers", InfFileName);
                if (File.Exists(candidatePath))
                {
                    var tempPath = Path.Combine(Path.GetTempPath(), $"candelta_{Guid.NewGuid():N}.inf");
                    await CopyFileAsync(candidatePath, tempPath);
                    return tempPath;
                }
                var parent = Directory.GetParent(searchDir);
                if (parent == null) break;
                searchDir = parent.FullName;
            }

            return null;
        }
        catch
        {
            return null;
        }
    }

    private static async Task CopyFileAsync(string source, string destination)
    {
        await using var sourceStream = File.OpenRead(source);
        await using var destStream = File.Create(destination);
        await sourceStream.CopyToAsync(destStream);
    }
}

/// <summary>
/// Result of a driver installation attempt.
/// </summary>
public record DriverInstallResult(bool Success, string Message);
