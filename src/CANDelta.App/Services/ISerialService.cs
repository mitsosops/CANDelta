using CANDelta.Core.Protocol;

namespace CANDelta.App.Services;

/// <summary>
/// Interface for serial communication with the CAN analyzer device.
/// </summary>
public interface ISerialService : IDisposable
{
    /// <summary>
    /// Event raised when a CAN frame is received from the device.
    /// </summary>
    event Action<CanFrame>? FrameReceived;

    /// <summary>
    /// Event raised when the connection state changes.
    /// </summary>
    event Action<bool>? ConnectionChanged;

    /// <summary>
    /// Event raised when an error occurs.
    /// </summary>
    event Action<string>? ErrorOccurred;

    /// <summary>
    /// Whether the device is currently connected.
    /// </summary>
    bool IsConnected { get; }

    /// <summary>
    /// Get list of available COM ports with USB device information.
    /// </summary>
    DetectedPort[] GetAvailablePorts();

    /// <summary>
    /// Connect to the device on the specified port.
    /// </summary>
    Task<bool> ConnectAsync(string portName);

    /// <summary>
    /// Disconnect from the device.
    /// </summary>
    void Disconnect();

    /// <summary>
    /// Send a ping command to verify connection.
    /// </summary>
    Task<bool> PingAsync();

    /// <summary>
    /// Get the device version.
    /// </summary>
    Task<DeviceVersion?> GetVersionAsync();

    /// <summary>
    /// Get the device status.
    /// </summary>
    Task<DeviceStatus?> GetStatusAsync();

    /// <summary>
    /// Start capturing CAN frames.
    /// </summary>
    Task<bool> StartCaptureAsync();

    /// <summary>
    /// Stop capturing CAN frames.
    /// </summary>
    Task<bool> StopCaptureAsync();

    /// <summary>
    /// Set the CAN bus speed.
    /// </summary>
    Task<bool> SetSpeedAsync(CanSpeed speed);

    /// <summary>
    /// Transmit a CAN frame.
    /// </summary>
    Task<bool> TransmitAsync(CanFrame frame);
}
