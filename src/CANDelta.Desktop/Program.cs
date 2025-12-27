using Avalonia;

namespace CANDelta.Desktop;

class Program
{
    [STAThread]
    public static void Main(string[] args) => BuildAvaloniaApp()
        .StartWithClassicDesktopLifetime(args);

    public static AppBuilder BuildAvaloniaApp()
        => AppBuilder.Configure<CANDelta.App.App>()
            .UsePlatformDetect()
            .LogToTrace();
}
