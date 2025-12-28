using System.Collections.Concurrent;
using System.Collections.ObjectModel;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using CANDelta.App.Services;
using CANDelta.Core.Protocol;
using System.Linq;

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
    private DispatcherTimer? _graphUpdateTimer;
    private DispatcherTimer? _portPollTimer;

    // FPS tracking
    private int _framesThisSecond;
    private DateTime _lastFpsUpdate = DateTime.UtcNow;

    // Track CANDelta detection state for auto-select
    private bool _hadCanDeltaDetected;

    // Constants
    private const int BatchSize = 100;
    private const double FadePerTick = 0.04; // ~1.5s fade at 60fps (90 ticks * 0.04 ≈ 3.6, but intensity starts < 1)
    private const double GraphWidth = 50.0;   // Width of sparkline graphs
    private const double GraphHeight = 16.0;  // Height of sparkline graphs

    [ObservableProperty]
    private string _title = "CANDelta - CAN Bus Monitor";

    [ObservableProperty]
    private bool _isConnected;

    [ObservableProperty]
    private bool _isCapturing;

    [ObservableProperty]
    private DetectedPort? _selectedPort;

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

    [ObservableProperty]
    private bool _showDriverInstallPrompt;

    [ObservableProperty]
    private bool _isInstallingDriver;

    public ObservableCollection<DetectedPort> AvailablePorts { get; } = new();
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

        // Graph update timer: update sparkline graphs at 10fps (100ms)
        _graphUpdateTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(100)
        };
        _graphUpdateTimer.Tick += OnGraphUpdateTick;
        _graphUpdateTimer.Start();

        // Port polling timer: check for new devices every 2 seconds
        _portPollTimer = new DispatcherTimer
        {
            Interval = TimeSpan.FromSeconds(2)
        };
        _portPollTimer.Tick += (_, _) => RefreshPorts();
        _portPollTimer.Start();
    }

    [RelayCommand]
    private void RefreshPorts()
    {
        var currentPorts = _serialService.GetAvailablePorts();
        var currentPortNames = new HashSet<string>(currentPorts.Select(p => p.PortName));
        var existingPortNames = new HashSet<string>(AvailablePorts.Select(p => p.PortName));

        // Check if CANDelta is now detected
        var canDeltaPort = currentPorts.FirstOrDefault(p => p.IsCanDelta);
        bool hasCanDeltaNow = canDeltaPort != null;

        // Skip update if ports haven't changed (but still check for CANDelta transition)
        bool portsChanged = !currentPortNames.SetEquals(existingPortNames);

        if (portsChanged)
        {
            // Remember current selection
            var previousSelection = SelectedPort?.PortName;

            AvailablePorts.Clear();
            foreach (var port in currentPorts)
            {
                AvailablePorts.Add(port);
            }

            // Restore previous selection if still available
            if (previousSelection != null)
            {
                var previousPort = AvailablePorts.FirstOrDefault(p => p.PortName == previousSelection);
                if (previousPort != null)
                {
                    SelectedPort = previousPort;
                }
            }

            // Select first available if nothing selected
            if (SelectedPort == null && AvailablePorts.Count > 0)
            {
                SelectedPort = AvailablePorts[0];
            }
        }

        // Auto-select CANDelta when it transitions from undetected to detected
        if (hasCanDeltaNow && !_hadCanDeltaDetected)
        {
            // Find the CANDelta port in the current list
            var canDeltaInList = AvailablePorts.FirstOrDefault(p => p.IsCanDelta);
            if (canDeltaInList != null)
            {
                SelectedPort = canDeltaInList;
            }
        }

        _hadCanDeltaDetected = hasCanDeltaNow;

        // Check if any CANDelta device needs driver installation (only prompt if not previously dismissed)
        if (!AppSettings.DriverPromptDismissed && System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Windows))
        {
            var needsDriver = currentPorts.FirstOrDefault(p => p.IsCanDelta && p.NeedsDriverInstall);
            if (needsDriver != null)
            {
                ShowDriverInstallPrompt = true;
            }
        }
    }

    [RelayCommand]
    private async Task InstallDriverAsync()
    {
        if (!System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Windows))
        {
            return;
        }

        ShowDriverInstallPrompt = false;
        IsInstallingDriver = true;
        StatusMessage = "Installing driver...";

        try
        {
            var result = await DriverInstaller.InstallDriverAsync();

            if (result.Success)
            {
                StatusMessage = result.Message;
                AppSettings.DriverPromptDismissed = true; // Don't prompt again after success
                // Refresh ports to pick up the new driver
                await Task.Delay(1000); // Give Windows time to re-enumerate
                RefreshPorts();
            }
            else
            {
                // Show warning, not error - the app works fine without the driver
                StatusMessage = "Driver not installed (optional). Device works normally but shows as 'USB Serial Device' in Device Manager.";
                AppSettings.DriverPromptDismissed = true; // Don't prompt again after failure
            }
        }
        finally
        {
            IsInstallingDriver = false;
        }
    }

    [RelayCommand]
    private void DismissDriverPrompt()
    {
        AppSettings.DriverPromptDismissed = true;
        ShowDriverInstallPrompt = false;
    }

    /// <summary>
    /// Manually trigger driver installation (for use from menu/settings).
    /// </summary>
    [RelayCommand]
    private async Task InstallDriverManuallyAsync()
    {
        // Reset the dismissed flag to allow the install to proceed
        await InstallDriverAsync();
    }

    [RelayCommand]
    private async Task ConnectAsync()
    {
        if (IsConnected)
        {
            _serialService.Disconnect();
            return;
        }

        if (SelectedPort == null)
        {
            StatusMessage = "Please select a COM port";
            return;
        }

        StatusMessage = $"Connecting to {SelectedPort.PortName}...";

        if (await _serialService.ConnectAsync(SelectedPort.PortName))
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

    private void OnGraphUpdateTick(object? sender, EventArgs e)
    {
        // Update graph points for all monitored IDs
        foreach (var item in MonitoredIds)
        {
            item.UpdateGraphPoints(GraphWidth, GraphHeight);
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
