using System;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using CANDelta.Core.Protocol;

namespace CANDelta.App.Services;

/// <summary>
/// Service for recording CAN frames to a PEAK TRC format trace file.
/// </summary>
public class TraceRecorderService : IDisposable
{
    private StreamWriter? _writer;
    private FileStream? _fileStream;
    private int _messageNumber;
    private readonly object _lock = new();
    private DateTime _startTime;
    private bool _disposed;

    /// <summary>
    /// Gets whether recording is currently active.
    /// </summary>
    public bool IsRecording => _writer != null;

    /// <summary>
    /// Gets the path to the current trace file, or null if not recording.
    /// </summary>
    public string? CurrentFilePath { get; private set; }

    /// <summary>
    /// Starts recording to a new trace file in PEAK TRC format.
    /// </summary>
    /// <param name="speed">The CAN bus speed for the trace header.</param>
    /// <returns>The path to the created trace file.</returns>
    public string StartRecording(CanSpeed speed)
    {
        lock (_lock)
        {
            if (IsRecording)
            {
                throw new InvalidOperationException("Recording is already in progress.");
            }

            // Create Traces subfolder in the directory of the exe
            var exeDirectory = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location) ?? Environment.CurrentDirectory;
            var tracesDirectory = Path.Combine(exeDirectory, "Traces");
            Directory.CreateDirectory(tracesDirectory);

            // Generate filename with timestamp
            var timestamp = DateTime.Now.ToString("yyyyMMddHHmmssfff", CultureInfo.InvariantCulture);
            var filename = $"CANDelta_{timestamp}.trc";
            CurrentFilePath = Path.Combine(tracesDirectory, filename);

            // Open file with FileShare.ReadWrite to allow other apps to read while we write
            _fileStream = new FileStream(
                CurrentFilePath,
                FileMode.Append,
                FileAccess.Write,
                FileShare.ReadWrite);
            _writer = new StreamWriter(_fileStream, Encoding.ASCII)
            {
                AutoFlush = true // Ensure data is written immediately
            };

            _messageNumber = 1;
            _startTime = DateTime.Now;

            // Write PEAK TRC v2.1 header
            WriteHeader(speed);

            return CurrentFilePath;
        }
    }

    /// <summary>
    /// Writes the PEAK TRC format header.
    /// </summary>
    private void WriteHeader(CanSpeed speed)
    {
        if (_writer == null) return;

        // PEAK TRC format v2.1 header
        _writer.WriteLine(";$FILEVERSION=2.1");
        _writer.WriteLine($";$STARTTIME={_startTime:yyyy-MM-dd HH:mm:ss.fff}");
        _writer.WriteLine(";$COLUMNS=N,O,T,B,I,d0,d1,d2,d3,d4,d5,d6,d7");
        _writer.WriteLine(";   N - Message Number");
        _writer.WriteLine(";   O - Time Offset (ms)");
        _writer.WriteLine(";   T - Message Type (Rx/Tx)");
        _writer.WriteLine(";   B - Bus (always 1)");
        _writer.WriteLine(";   I - CAN ID (hex)");
        _writer.WriteLine(";   d0..d7 - Data bytes (hex)");
        _writer.WriteLine($";$BITRATE={GetBitrate(speed)}");
        _writer.WriteLine(";");
    }

    /// <summary>
    /// Records a CAN frame to the trace file.
    /// </summary>
    /// <param name="frame">The CAN frame to record.</param>
    public void RecordFrame(CanFrame frame)
    {
        lock (_lock)
        {
            if (_writer == null) return;

            // Calculate time offset in milliseconds from start
            var timeOffset = (DateTime.Now - _startTime).TotalMilliseconds;

            // Format: N O T B I d0 d1 d2 d3 d4 d5 d6 d7
            // N = message number
            // O = time offset in ms
            // T = Rx (receive)
            // B = bus number (1)
            // I = CAN ID in hex
            // d0-d7 = data bytes in hex

            var sb = new StringBuilder();
            sb.Append($"{_messageNumber,7}");
            sb.Append($" {timeOffset,13:F1}");
            sb.Append("  Rx");
            sb.Append("  1");
            
            // Format CAN ID - extended IDs use 8 hex digits with 'h' suffix
            if (frame.IsExtended)
            {
                sb.Append($"  {frame.Id:X8}h");
            }
            else
            {
                sb.Append($"  {frame.Id:X4}    ");
            }

            // Format data bytes
            var dataLength = Math.Min(frame.Dlc, frame.Data?.Length ?? 0);
            for (int i = 0; i < 8; i++)
            {
                if (i < dataLength && frame.Data != null)
                {
                    sb.Append($" {frame.Data[i]:X2}");
                }
            }

            _writer.WriteLine(sb.ToString());
            _messageNumber++;
        }
    }

    /// <summary>
    /// Stops recording and closes the trace file.
    /// </summary>
    public void StopRecording()
    {
        lock (_lock)
        {
            if (_writer != null)
            {
                _writer.Flush();
                _writer.Dispose();
                _writer = null;
            }

            if (_fileStream != null)
            {
                _fileStream.Dispose();
                _fileStream = null;
            }

            CurrentFilePath = null;
            _messageNumber = 0;
        }
    }

    /// <summary>
    /// Gets the bitrate value for the trace header.
    /// </summary>
    private static int GetBitrate(CanSpeed speed)
    {
        return speed switch
        {
            CanSpeed.Speed125Kbps => 125000,
            CanSpeed.Speed250Kbps => 250000,
            CanSpeed.Speed500Kbps => 500000,
            CanSpeed.Speed1Mbps => 1000000,
            _ => 500000
        };
    }

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    protected virtual void Dispose(bool disposing)
    {
        if (_disposed) return;

        if (disposing)
        {
            StopRecording();
        }

        _disposed = true;
    }
}
