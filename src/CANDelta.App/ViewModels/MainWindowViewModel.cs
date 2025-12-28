using System.Collections.Concurrent;
using System.Collections.ObjectModel;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using CANDelta.App.Services;
using CANDelta.Core.Protocol;

namespace CANDelta.App.ViewModels;

public partial class MainWindowViewModel : ObservableObject
{
    private readonly ISerialService _serialService;

    // Frame processing
    private readonly ConcurrentQueue<CanFrame> _pendingFrames = new();
    private readonly Dictionary<uint, MonitoredCanId> _monitoredIdsLookup = new();
    private readonly HashSet<MonitoredCanId> _activeAnimations = new();

    // Timers
    private DispatcherTimer? _batchUpdateTimer;
    private DispatcherTimer? _fadeTimer;

    // FPS tracking
    private int _framesThisSecond;
    private DateTime _lastFpsUpdate = DateTime.UtcNow;

    // Constants
    private const int BatchSize = 100;
    private const double FadePerTick = 0.04; // ~1.5s fade at 60fps (90 ticks * 0.04 ≈ 3.6, but intensity starts < 1)

    [ObservableProperty]
    private string _title = "CANDelta - CAN Bus Monitor";

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isCapturing;

    [ObservableProperty]
    private string _selectedPort = "";

    [ObservableProperty]
    private CanSpeed _selectedSpeed = CanSpeed.Speed500Kbps;

    [ObservableProperty]
    private string _statusMessage = "Disconnected";

    [ObservableProperty]
    private int _frameCount;

    [ObservableProperty]
    private int _uniqueIdCount;

    [ObservableProperty]
    private int _framesPerSecond;

    [ObservableProperty]
    private ColorTheme _selectedTheme = ColorTheme.AvailableThemes[0]; // Cyan

    public ObservableCollection<string> AvailablePorts { get; } = new();
    public ObservableCollection<MonitoredCanId> MonitoredIds { get; } = new();

    public IEnumerable<CanSpeed> AvailableSpeeds { get; } = Enum.GetValues<CanSpeed>();
    public ColorTheme[] AvailableThemes { get; } = ColorTheme.AvailableThemes;

    public MainWindowViewModel(ISerialService serialService)
    {
        _serialService = serialService;
        _serialService.FrameReceived += OnFrameReceived;
        _serialService.ConnectionChanged += OnConnectionChanged;
        _serialService.ErrorOccurred += OnError;

        InitializeTimers();
        RefreshPorts();
    }

    private void InitializeTimers()
    {
        // Batch update timer: process queued frames at 60fps
        _batchUpdateTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(16)
        };
        _batchUpdateTimer.Tick += OnBatchUpdateTick;
        _batchUpdateTimer.Start();

        // Fade timer: animate color fade-back at 60fps
        _fadeTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(16)
        };
        _fadeTimer.Tick += OnFadeTimerTick;
        _fadeTimer.Start();
    }

    [RelayCommand]
    private void RefreshPorts()
    {
        AvailablePorts.Clear();
        foreach (var port in _serialService.GetAvailablePorts())
        {
            AvailablePorts.Add(port);
        }

        if (AvailablePorts.Count > 0 && string.IsNullOrEmpty(SelectedPort))
        {
            SelectedPort = AvailablePorts[0];
        }
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (IsConnected)
        {
            _serialService.Disconnect();
            return;
        }

        if (string.IsNullOrEmpty(SelectedPort))
        {
            StatusMessage = "Please select a COM port";
            return;
        }

        StatusMessage = $"Connecting to {SelectedPort}...";

        if (await _serialService.ConnectAsync(SelectedPort))
        {
            var version = await _serialService.GetVersionAsync();
            StatusMessage = version != null
                ? $"Connected - Firmware {version}"
                : "Connected";

            await _serialService.SetSpeedAsync(SelectedSpeed);
        }
        else
        {
            StatusMessage = "Connection failed";
        }
    }

    [RelayCommand]
    private async Task ToggleCaptureAsync()
    {
        if (!IsConnected) return;

        if (IsCapturing)
        {
            await _serialService.StopCaptureAsync();
            IsCapturing = false;
            StatusMessage = $"Capture stopped - {UniqueIdCount} IDs, {FrameCount} frames";
        }
        else
        {
            if (await _serialService.StartCaptureAsync())
            {
                IsCapturing = true;
                StatusMessage = "Capturing...";
            }
        }
    }

    [RelayCommand]
    private void ClearMonitor()
    {
        // Clear pending frames
        while (_pendingFrames.TryDequeue(out _)) { }

        // Clear tracked IDs
        _monitoredIdsLookup.Clear();
        _activeAnimations.Clear();
        MonitoredIds.Clear();

        // Reset counters
        FrameCount = 0;
        UniqueIdCount = 0;
        FramesPerSecond = 0;
        _framesThisSecond = 0;

        StatusMessage = "Monitor cleared";
    }

    private void OnFrameReceived(CanFrame frame)
    {
        // Thread-safe enqueue (called from serial thread)
        _pendingFrames.Enqueue(frame);
    }

    private void OnBatchUpdateTick(object? sender, EventArgs e)
    {
        // Process up to BatchSize frames per tick
        int processed = 0;
        var themeColor = SelectedTheme.AlertColor;

        while (processed < BatchSize && _pendingFrames.TryDequeue(out var frame))
        {
            ProcessFrame(frame, themeColor);
            processed++;
        }

        // Update FPS counter
        _framesThisSecond += processed;
        var now = DateTime.UtcNow;
        if ((now - _lastFpsUpdate).TotalSeconds >= 1.0)
        {
            FramesPerSecond = _framesThisSecond;
            _framesThisSecond = 0;
            _lastFpsUpdate = now;
        }
    }

    private void ProcessFrame(CanFrame frame, Avalonia.Media.Color themeColor)
    {
        FrameCount++;

        // Look up or create monitored ID
        if (!_monitoredIdsLookup.TryGetValue(frame.Id, out var monitoredId))
        {
            monitoredId = new MonitoredCanId(frame.Id, frame.IsExtended);
            _monitoredIdsLookup[frame.Id] = monitoredId;
            InsertSorted(monitoredId);
            UniqueIdCount = MonitoredIds.Count;
        }

        // Update bytes and mark as animating
        monitoredId.UpdateFromFrame(frame, themeColor);
        _activeAnimations.Add(monitoredId);
    }

    private void InsertSorted(MonitoredCanId item)
    {
        // Binary search for insertion point
        int lo = 0, hi = MonitoredIds.Count;
        while (lo < hi)
        {
            int mid = (lo + hi) / 2;
            if (MonitoredIds[mid].Id < item.Id)
                lo = mid + 1;
            else
                hi = mid;
        }
        MonitoredIds.Insert(lo, item);
    }

    private void OnFadeTimerTick(object? sender, EventArgs e)
    {
        if (_activeAnimations.Count == 0) return;

        var themeColor = SelectedTheme.AlertColor;
        var toRemove = new List<MonitoredCanId>();

        foreach (var item in _activeAnimations)
        {
            item.ApplyFade(FadePerTick, themeColor);
            if (!item.HasActiveAnimation())
            {
                toRemove.Add(item);
            }
        }

        foreach (var item in toRemove)
        {
            _activeAnimations.Remove(item);
        }
    }

    partial void OnSelectedThemeChanged(ColorTheme value)
    {
        // Reapply colors when theme changes
        var color = value.AlertColor;
        foreach (var item in _activeAnimations)
        {
            item.ApplyFade(0, color); // Just reapply colors without fading
        }
    }

    private void OnConnectionChanged(bool connected)
    {
        Dispatcher.UIThread.Post(() =>
        {
            IsConnected = connected;
            if (!connected)
            {
                IsCapturing = false;
                StatusMessage = "Disconnected";
            }
        });
    }

    private void OnError(string message)
    {
        Dispatcher.UIThread.Post(() =>
        {
            StatusMessage = $"Error: {message}";
        });
    }
}
