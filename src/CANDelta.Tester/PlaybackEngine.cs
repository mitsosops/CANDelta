using System.Diagnostics;
using CANDelta.Core.Protocol;

namespace CANDelta.Tester;

/// <summary>
/// Handles timing-accurate playback of CAN frames from a trace file.
/// </summary>
public class PlaybackEngine
{
    private readonly List<TrcFrame> _frames;
    private readonly double _speedMultiplier;
    private readonly bool _loop;
    private readonly bool _verbose;
    private readonly bool _showStats;
    private readonly Action<CanFrame> _sendFrame;

    private volatile bool _captureActive;
    private volatile bool _stopRequested;
    private Thread? _playbackThread;
    private readonly object _lock = new();

    private ulong _baseTimestampUs;
    private uint _framesSent;
    private uint _loopCount;

    public PlaybackEngine(
        List<TrcFrame> frames,
        double speedMultiplier,
        bool loop,
        bool verbose,
        bool showStats,
        Action<CanFrame> sendFrame)
    {
        _frames = frames;
        _speedMultiplier = speedMultiplier;
        _loop = loop;
        _verbose = verbose;
        _showStats = showStats;
        _sendFrame = sendFrame;
    }

    /// <summary>
    /// Number of frames sent since capture started.
    /// </summary>
    public uint FramesSent => _framesSent;

    /// <summary>
    /// Whether capture is currently active.
    /// </summary>
    public bool CaptureActive => _captureActive;

    /// <summary>
    /// Start frame playback. Called when START_CAPTURE is received.
    /// </summary>
    public void StartCapture()
    {
        lock (_lock)
        {
            if (_captureActive)
                return;

            _captureActive = true;
            _stopRequested = false;
            _framesSent = 0;
            _baseTimestampUs = (ulong)(Stopwatch.GetTimestamp() * 1_000_000.0 / Stopwatch.Frequency);

            _playbackThread = new Thread(PlaybackLoop)
            {
                Name = "PlaybackEngine",
                IsBackground = true
            };
            _playbackThread.Start();
        }
    }

    /// <summary>
    /// Stop frame playback. Called when STOP_CAPTURE is received.
    /// </summary>
    public void StopCapture()
    {
        lock (_lock)
        {
            if (!_captureActive)
                return;

            _stopRequested = true;
            _captureActive = false;
        }

        _playbackThread?.Join(1000);
        _playbackThread = null;
    }

    /// <summary>
    /// Shutdown the playback engine completely.
    /// </summary>
    public void Shutdown()
    {
        StopCapture();
    }

    private void PlaybackLoop()
    {
        if (_frames.Count == 0)
        {
            Console.WriteLine("No frames to play back.");
            return;
        }

        var statsStartTime = Stopwatch.GetTimestamp();
        var statsFrameCount = 0u;
        var lastStatsUpdate = statsStartTime;

        do
        {
            _loopCount++;
            var startTime = Stopwatch.GetTimestamp();
            var firstFrameTime = _frames[0].TimestampSeconds;

            foreach (var trcFrame in _frames)
            {
                if (_stopRequested)
                    return;

                // Calculate when this frame should be sent relative to playback start
                var relativeTime = trcFrame.TimestampSeconds - firstFrameTime;

                // Apply speed multiplier (0 = no delay, otherwise divide by multiplier)
                if (_speedMultiplier > 0)
                {
                    var targetElapsed = relativeTime / _speedMultiplier;
                    var targetTicks = startTime + (long)(targetElapsed * Stopwatch.Frequency);

                    // Spin-wait until target time
                    while (Stopwatch.GetTimestamp() < targetTicks)
                    {
                        if (_stopRequested)
                            return;

                        // Small sleep to reduce CPU usage, but keep timing reasonably accurate
                        if (targetTicks - Stopwatch.GetTimestamp() > Stopwatch.Frequency / 100)
                        {
                            Thread.Sleep(1);
                        }
                        else
                        {
                            Thread.SpinWait(10);
                        }
                    }
                }

                // Calculate timestamp (microseconds since capture start)
                var nowUs = (ulong)(Stopwatch.GetTimestamp() * 1_000_000.0 / Stopwatch.Frequency);
                var timestampUs = nowUs - _baseTimestampUs + (ulong)(trcFrame.TimestampSeconds * 1_000_000);

                var canFrame = new CanFrame
                {
                    Id = trcFrame.CanId,
                    Data = trcFrame.Data,
                    Dlc = (byte)trcFrame.Data.Length,
                    IsExtended = trcFrame.CanId > 0x7FF,
                    IsRtr = false,
                    TimestampUs = timestampUs
                };

                _sendFrame(canFrame);
                _framesSent++;
                statsFrameCount++;

                if (_verbose)
                {
                    Console.WriteLine($"TX: {canFrame}");
                }

                // Show stats every second
                if (_showStats)
                {
                    var now = Stopwatch.GetTimestamp();
                    var elapsed = (now - lastStatsUpdate) / (double)Stopwatch.Frequency;
                    if (elapsed >= 1.0)
                    {
                        var fps = statsFrameCount / elapsed;
                        var totalElapsed = (now - statsStartTime) / (double)Stopwatch.Frequency;
                        Console.Write($"\rFrames: {_framesSent,8} | FPS: {fps,6:F0} | Loop: {_loopCount,4} | Elapsed: {totalElapsed,6:F1}s   ");
                        statsFrameCount = 0;
                        lastStatsUpdate = now;
                    }
                }
            }

            if (_loop && !_stopRequested)
            {
                if (!_showStats)
                {
                    Console.WriteLine($"Loop complete, {_framesSent} frames sent. Restarting...");
                }
                // Reset timestamp base for new loop
                _baseTimestampUs = (ulong)(Stopwatch.GetTimestamp() * 1_000_000.0 / Stopwatch.Frequency);
            }

        } while (_loop && !_stopRequested);

        if (_showStats)
        {
            Console.WriteLine(); // New line after stats
        }
        Console.WriteLine($"Playback complete. Total frames sent: {_framesSent}");
        _captureActive = false;
    }
}
