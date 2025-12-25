using System.Collections.ObjectModel;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using CANDelta.App.Services;
using CANDelta.Core.Analysis;
using CANDelta.Core.Models;
using CANDelta.Core.Protocol;

namespace CANDelta.App.ViewModels;

public partial class MainWindowViewModel : ObservableObject
{
    private readonly ISerialService _serialService;
    private readonly DeltaAnalyzer _analyzer = new();
    private Trace? _currentTrace;

    [ObservableProperty]
    private string _title = "CANDelta - CAN Bus Analyzer";

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
    private Behaviour? _controlBehaviour;

    [ObservableProperty]
    private Behaviour? _testBehaviour;

    [ObservableProperty]
    private DeltaResult? _deltaResult;

    public ObservableCollection<string> AvailablePorts { get; } = new();
    public ObservableCollection<CanFrame> LiveFrames { get; } = new();
    public ObservableCollection<Behaviour> Behaviours { get; } = new();
    public ObservableCollection<DeltaFrame> DeltaFrames { get; } = new();

    public IEnumerable<CanSpeed> AvailableSpeeds { get; } = Enum.GetValues<CanSpeed>();

    public MainWindowViewModel(ISerialService serialService)
    {
        _serialService = serialService;
        _serialService.FrameReceived += OnFrameReceived;
        _serialService.ConnectionChanged += OnConnectionChanged;
        _serialService.ErrorOccurred += OnError;

        RefreshPorts();
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
            StatusMessage = "Capture stopped";

            if (_currentTrace != null)
            {
                _currentTrace.EndTime = DateTime.UtcNow;
            }
        }
        else
        {
            LiveFrames.Clear();
            FrameCount = 0;

            _currentTrace = new Trace { Speed = SelectedSpeed };

            if (await _serialService.StartCaptureAsync())
            {
                IsCapturing = true;
                StatusMessage = "Capturing...";
            }
        }
    }

    [RelayCommand]
    private void SaveTraceToControl()
    {
        if (_currentTrace == null || _currentTrace.Frames.Count == 0) return;

        ControlBehaviour ??= new Behaviour { Name = "Control", IsControl = true };
        ControlBehaviour.Traces.Add(_currentTrace);

        StatusMessage = $"Saved trace to Control ({ControlBehaviour.Traces.Count} traces, {_currentTrace.Frames.Count} frames)";
        _currentTrace = null;
    }

    [RelayCommand]
    private void SaveTraceToTest()
    {
        if (_currentTrace == null || _currentTrace.Frames.Count == 0) return;

        TestBehaviour ??= new Behaviour { Name = "Test", IsControl = false };
        TestBehaviour.Traces.Add(_currentTrace);

        StatusMessage = $"Saved trace to Test ({TestBehaviour.Traces.Count} traces, {_currentTrace.Frames.Count} frames)";
        _currentTrace = null;
    }

    [RelayCommand]
    private void AnalyzeDelta()
    {
        if (ControlBehaviour == null || TestBehaviour == null)
        {
            StatusMessage = "Need both Control and Test behaviours";
            return;
        }

        if (ControlBehaviour.Traces.Count == 0 || TestBehaviour.Traces.Count == 0)
        {
            StatusMessage = "Need at least one trace in each behaviour";
            return;
        }

        DeltaResult = _analyzer.Analyze(ControlBehaviour, TestBehaviour);

        DeltaFrames.Clear();
        foreach (var frame in DeltaResult.UniqueToTest)
        {
            DeltaFrames.Add(frame);
        }

        StatusMessage = DeltaResult.Summary;
    }

    [RelayCommand]
    private void ClearAll()
    {
        ControlBehaviour = null;
        TestBehaviour = null;
        DeltaResult = null;
        DeltaFrames.Clear();
        LiveFrames.Clear();
        FrameCount = 0;
        _currentTrace = null;
        StatusMessage = "Cleared all data";
    }

    private void OnFrameReceived(CanFrame frame)
    {
        // Must dispatch to UI thread
        Avalonia.Threading.Dispatcher.UIThread.Post(() =>
        {
            LiveFrames.Add(frame);
            FrameCount = LiveFrames.Count;

            _currentTrace?.Frames.Add(frame);

            // Keep only last 1000 frames in live view
            while (LiveFrames.Count > 1000)
            {
                LiveFrames.RemoveAt(0);
            }
        });
    }

    private void OnConnectionChanged(bool connected)
    {
        Avalonia.Threading.Dispatcher.UIThread.Post(() =>
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
        Avalonia.Threading.Dispatcher.UIThread.Post(() =>
        {
            StatusMessage = $"Error: {message}";
        });
    }
}
