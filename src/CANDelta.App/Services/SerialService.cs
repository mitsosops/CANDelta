using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Runtime.Versioning;
using System.Text.RegularExpressions;
using CANDelta.Core.Protocol;

namespace CANDelta.App.Services;

/// <summary>
/// Serial communication service for the CAN analyzer device.
/// </summary>
public class SerialService : ISerialService
{
    private SerialPort? _port;
    private readonly ProtocolParser _parser = new();
    private readonly SemaphoreSlim _commandLock = new(1, 1);
    private TaskCompletionSource<(ResponseCode, byte[])>? _pendingResponse;
    private CancellationTokenSource? _readCts;

    public event Action<CanFrame>? FrameReceived;
    public event Action<bool>? ConnectionChanged;
    public event Action<string>? ErrorOccurred;

    public bool IsConnected => _port?.IsOpen ?? false;

    public SerialService()
    {
        _parser.FrameReceived += frame => FrameReceived?.Invoke(frame);
        _parser.ResponseReceived += OnResponseReceived;
    }

    public DetectedPort[] GetAvailablePorts()
    {
        var portNames = SerialPort.GetPortNames();

        if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
        {
            return GetWindowsPortsWithUsbInfo(portNames);
        }

        // Fallback for non-Windows: return ports without USB info
        return portNames.Select(p => new DetectedPort(p)).ToArray();
    }

    [SupportedOSPlatform("windows")]
    private static DetectedPort[] GetWindowsPortsWithUsbInfo(string[] portNames)
    {
        var result = new List<DetectedPort>();
        // Maps port name to (isCanDelta, needsDriverInstall)
        var portInfo = new Dictionary<string, (bool isCanDelta, bool needsDriver)>(StringComparer.OrdinalIgnoreCase);

        try
        {
            // Query WMI for USB serial devices
            using var searcher = new System.Management.ManagementObjectSearcher(
                "SELECT * FROM Win32_PnPEntity WHERE Caption LIKE '%(COM%'");

            foreach (System.Management.ManagementObject obj in searcher.Get())
            {
                var caption = obj["Caption"]?.ToString();
                var pnpDeviceId = obj["PNPDeviceID"]?.ToString();

                if (string.IsNullOrEmpty(caption) || string.IsNullOrEmpty(pnpDeviceId)) continue;

                // Extract COM port from caption (e.g., "USB Serial Device (COM7)")
                var match = Regex.Match(caption, @"\((COM\d+)\)");
                if (!match.Success) continue;

                var portName = match.Groups[1].Value;

                // Check if this is a CANDelta device by VID/PID
                bool isCanDelta = pnpDeviceId.Contains(DetectedPort.CanDeltaVidPid, StringComparison.OrdinalIgnoreCase);

                if (isCanDelta)
                {
                    // Check if driver is installed (caption shows "CANDelta" instead of "USB Serial Device")
                    bool driverInstalled = caption.Contains("CANDelta", StringComparison.OrdinalIgnoreCase);
                    portInfo[portName] = (true, !driverInstalled);
                }
            }
        }
        catch
        {
            // WMI query failed, fall back to simple port names
        }

        foreach (var portName in portNames.OrderBy(p => p))
        {
            if (portInfo.TryGetValue(portName, out var info))
            {
                result.Add(new DetectedPort(portName, info.isCanDelta, info.needsDriver));
            }
            else
            {
                result.Add(new DetectedPort(portName));
            }
        }

        return result.ToArray();
    }

    public async Task<bool> ConnectAsync(string portName)
    {
        try
        {
            Disconnect();

            _port = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 1000,
                WriteTimeout = 1000,
                DtrEnable = true,
                RtsEnable = true
            };

            _port.Open();
            _readCts = new CancellationTokenSource();

            // Start background read task
            _ = Task.Run(() => ReadLoop(_readCts.Token));

            // Small delay for device to initialize
            await Task.Delay(100);

            // Verify connection with ping
            var pingOk = await PingAsync();
            if (!pingOk)
            {
                Disconnect();
                return false;
            }

            ConnectionChanged?.Invoke(true);
            return true;
        }
        catch (Exception ex)
        {
            ErrorOccurred?.Invoke($"Connection failed: {ex.Message}");
            Disconnect();
            return false;
        }
    }

    public void Disconnect()
    {
        _readCts?.Cancel();
        _readCts?.Dispose();
        _readCts = null;

        if (_port?.IsOpen == true)
        {
            try
            {
                _port.Close();
            }
            catch
            {
                // Ignore close errors
            }
        }

        _port?.Dispose();
        _port = null;

        ConnectionChanged?.Invoke(false);
    }

    private async Task ReadLoop(CancellationToken ct)
    {
        var buffer = new byte[1024];

        while (!ct.IsCancellationRequested && _port?.IsOpen == true)
        {
            try
            {
                var bytesToRead = _port.BytesToRead;
                if (bytesToRead > 0)
                {
                    var count = Math.Min(bytesToRead, buffer.Length);
                    var read = _port.Read(buffer, 0, count);
                    if (read > 0)
                    {
                        _parser.ProcessBytes(buffer, 0, read);
                    }
                }
                else
                {
                    await Task.Delay(1, ct);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                ErrorOccurred?.Invoke($"Read error: {ex.Message}");
                await Task.Delay(100, ct);
            }
        }
    }

    private void OnResponseReceived(ResponseCode code, byte[] data)
    {
        _pendingResponse?.TrySetResult((code, data));
    }

    private async Task<(ResponseCode, byte[])?> SendCommandAsync(byte[] command, int timeoutMs = 1000)
    {
        if (_port?.IsOpen != true) return null;

        await _commandLock.WaitAsync();
        try
        {
            _pendingResponse = new TaskCompletionSource<(ResponseCode, byte[])>();

            _port.Write(command, 0, command.Length);

            using var cts = new CancellationTokenSource(timeoutMs);
            cts.Token.Register(() => _pendingResponse.TrySetCanceled());

            try
            {
                return await _pendingResponse.Task;
            }
            catch (OperationCanceledException)
            {
                return null;
            }
        }
        finally
        {
            _pendingResponse = null;
            _commandLock.Release();
        }
    }

    public async Task<bool> PingAsync()
    {
        var cmd = ProtocolParser.SerializeCommand(CommandOpcode.Ping);
        var response = await SendCommandAsync(cmd);
        return response?.Item1 == ResponseCode.Ack;
    }

    public async Task<DeviceVersion?> GetVersionAsync()
    {
        var cmd = ProtocolParser.SerializeCommand(CommandOpcode.GetVersion);
        var response = await SendCommandAsync(cmd);
        if (response?.Item1 == ResponseCode.Version)
        {
            return ProtocolParser.ParseVersionResponse(response.Value.Item2);
        }
        return null;
    }

    public async Task<DeviceStatus?> GetStatusAsync()
    {
        var cmd = ProtocolParser.SerializeCommand(CommandOpcode.GetStatus);
        var response = await SendCommandAsync(cmd);
        if (response?.Item1 == ResponseCode.Status)
        {
            return ProtocolParser.ParseStatusResponse(response.Value.Item2);
        }
        return null;
    }

    public async Task<bool> StartCaptureAsync()
    {
        var cmd = ProtocolParser.SerializeCommand(CommandOpcode.StartCapture);
        var response = await SendCommandAsync(cmd);
        return response?.Item1 == ResponseCode.Ack;
    }

    public async Task<bool> StopCaptureAsync()
    {
        var cmd = ProtocolParser.SerializeCommand(CommandOpcode.StopCapture);
        var response = await SendCommandAsync(cmd);
        return response?.Item1 == ResponseCode.Ack;
    }

    public async Task<bool> SetSpeedAsync(CanSpeed speed)
    {
        var cmd = ProtocolParser.CreateSetSpeedCommand(speed);
        var response = await SendCommandAsync(cmd);
        return response?.Item1 == ResponseCode.Ack;
    }

    public async Task<bool> TransmitAsync(CanFrame frame)
    {
        var cmd = ProtocolParser.CreateTransmitCommand(frame);
        var response = await SendCommandAsync(cmd);
        return response?.Item1 == ResponseCode.Ack;
    }

    public void Dispose()
    {
        Disconnect();
        _commandLock.Dispose();
    }
}
