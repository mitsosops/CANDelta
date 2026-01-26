namespace CANDelta.Tester;

class Program
{
    static int Main(string[] args)
    {
        string? port = null;
        string? file = null;
        double speed = 1.0;
        bool loop = false;
        bool verbose = false;
        bool showStats = false;

        // Parse command line arguments
        for (int i = 0; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "-p":
                case "--port":
                    if (i + 1 < args.Length) port = args[++i];
                    break;
                case "-f":
                case "--file":
                    if (i + 1 < args.Length) file = args[++i];
                    break;
                case "-s":
                case "--speed":
                    if (i + 1 < args.Length && double.TryParse(args[++i], out var s)) speed = s;
                    break;
                case "-l":
                case "--loop":
                    loop = true;
                    break;
                case "-v":
                case "--verbose":
                    verbose = true;
                    break;
                case "--stats":
                    showStats = true;
                    break;
                case "-h":
                case "--help":
                    PrintUsage();
                    return 0;
            }
        }

        if (string.IsNullOrEmpty(port))
        {
            Console.WriteLine("Error: COM port is required.");
            PrintUsage();
            return 1;
        }

        // Parse trace file if provided
        List<TrcFrame>? frames = null;
        if (!string.IsNullOrEmpty(file))
        {
            if (!File.Exists(file))
            {
                Console.WriteLine($"Error: File not found: {file}");
                return 1;
            }

            try
            {
                frames = TrcParser.Parse(file);
                Console.WriteLine($"Loaded {frames.Count} frames from {file}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error parsing trace file: {ex.Message}");
                return 1;
            }
        }
        else
        {
            Console.WriteLine("No trace file specified - running in idle mode (no frames will be sent)");
        }

        // Create device first
        using var device = new VirtualDevice(port, verbose);

        // Create playback engine if we have frames, and attach it to device
        if (frames != null && frames.Count > 0)
        {
            var playbackEngine = new PlaybackEngine(frames, speed, loop, verbose, showStats, device.SendCanFrame);
            device.SetPlaybackEngine(playbackEngine);
        }

        // Set up Ctrl+C handler
        var exitEvent = new ManualResetEvent(false);
        Console.CancelKeyPress += (_, e) =>
        {
            e.Cancel = true;
            Console.WriteLine("\nShutting down...");
            exitEvent.Set();
        };

        try
        {
            device.Start();

            Console.WriteLine();
            Console.WriteLine("Virtual CANDelta device ready.");
            Console.WriteLine($"  Port: {port}");
            Console.WriteLine($"  Trace file: {file ?? "(none)"}");
            Console.WriteLine($"  Speed multiplier: {speed}x{(speed == 0 ? " (max speed)" : "")}");
            Console.WriteLine($"  Loop: {(loop ? "enabled" : "disabled")}");
            Console.WriteLine($"  Stats: {(showStats ? "enabled" : "disabled")}");
            Console.WriteLine($"  Verbose: {(verbose ? "enabled" : "disabled")}");
            Console.WriteLine();
            Console.WriteLine("Press Ctrl+C to exit.");
            Console.WriteLine();

            // Wait for exit signal
            exitEvent.WaitOne();
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Error: {ex.Message}");
            return 1;
        }

        return 0;
    }

    static void PrintUsage()
    {
        Console.WriteLine(@"
CANDelta.Tester - Virtual CANDelta device for UI testing

Usage:
  CANDelta.Tester -p <port> [-f <file>] [-s <speed>] [-l] [-v]

Options:
  -p, --port <port>    COM port to listen on (required)
  -f, --file <path>    .trc trace file to play back
  -s, --speed <mult>   Playback speed multiplier (default: 1.0, 0 = max speed)
  -l, --loop           Loop playback continuously
  --stats              Show live statistics (frames, FPS, loop count)
  -v, --verbose        Show individual frame details
  -h, --help           Show this help message

Examples:
  # Listen on COM10, play trace.trc in a loop with stats
  CANDelta.Tester -p COM10 -f trace.trc --loop --stats

  # Listen on COM10, no trace (idle mode)
  CANDelta.Tester -p COM10

  # Play at 2x speed with verbose frame output
  CANDelta.Tester -p COM10 -f trace.trc -s 2.0 -v

Virtual COM Port Setup:
  Windows: Install com0com, create COM10<->COM11 pair
  Linux/macOS: Use socat to create virtual port pair
");
    }
}
