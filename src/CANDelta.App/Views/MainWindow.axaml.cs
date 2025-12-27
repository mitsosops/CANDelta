using Avalonia.Controls;
using Avalonia.Data.Converters;
using Avalonia.Media;

namespace CANDelta.App.Views
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }
    }
}

namespace CANDelta.App.ViewModels
{
    public partial class MainWindowViewModel
    {
        public static FuncValueConverter<bool, string> ConnectButtonConverter { get; } =
            new(isConnected => isConnected ? "Disconnect" : "Connect");

        public static FuncValueConverter<bool, string> CaptureButtonConverter { get; } =
            new(isCapturing => isCapturing ? "Stop" : "Capture");

        public static FuncValueConverter<bool, IBrush> CaptureLedConverter { get; } =
            new(isCapturing => isCapturing ? Brushes.Red : Brushes.Gray);

        public static FuncValueConverter<byte[]?, string> HexConverter { get; } =
            new(data => data != null ? Convert.ToHexString(data) : "");
    }
}
