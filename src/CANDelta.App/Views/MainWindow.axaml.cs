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

        public static FuncValueConverter<Color, IBrush> ColorToBrushConverter { get; } =
            new(color => new SolidColorBrush(color));

        public static FuncValueConverter<int, IBrush> FpsColorConverter { get; } =
            new(fps => fps switch
            {
                >= 500 => Brushes.Lime,
                >= 100 => Brushes.Yellow,
                > 0 => Brushes.Orange,
                _ => Brushes.Gray
            });
    }
}
