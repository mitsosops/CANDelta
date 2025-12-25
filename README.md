# CANDelta

CAN bus delta analyzer for reverse engineering vehicle networks. Captures CAN traces from control and test behaviours, then computes the delta to isolate feature-specific frames.

## Components

- **Firmware**: RP2040-based CAN analyzer (Adafruit Feather RP2040 CAN with MCP25625)
- **Desktop App**: Cross-platform Avalonia UI application (Windows, Linux, macOS)

## Hardware Requirements

- [Adafruit Feather RP2040 CAN](https://www.adafruit.com/product/5724) (or compatible RP2040 + MCP2515/MCP25625)
- USB-C cable
- CAN bus connection (CANH, CANL, GND)

## Building

### Firmware

Requires [Pico SDK](https://github.com/raspberrypi/pico-sdk) installed.

```bash
cd firmware
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk
make
```

Flash the resulting `candelta.uf2` to the RP2040 (hold BOOTSEL while connecting USB).

### Desktop Application

Requires [.NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0).

```bash
dotnet build
dotnet run --project src/CANDelta.Desktop
```

## Usage

1. Connect the CAN analyzer to your vehicle's CAN bus
2. Launch the desktop application
3. Select the COM port and CAN speed
4. **Control behaviour**: Capture multiple traces of normal operation
5. **Test behaviour**: Capture multiple traces while activating the target feature
6. Click "Analyze Delta" to find frames unique to the test behaviour

## Wire Protocol

Binary protocol over USB CDC (virtual COM port):
- STX (0x02) + Command + Params + ETX (0x03)
- CAN frames streamed with 64-bit microsecond timestamps

See `docs/protocol.md` for details.

## Project Structure

```
CANDelta/
├── firmware/           # RP2040 Pico SDK firmware
│   ├── src/
│   │   ├── can/       # MCP2515 driver
│   │   ├── usb/       # USB CDC communication
│   │   └── protocol/  # Command handling
│   └── include/
├── src/
│   ├── CANDelta.Core/  # Shared library (protocol, analysis)
│   ├── CANDelta.App/   # Avalonia UI application
│   └── CANDelta.Desktop/ # Desktop entry point
└── tests/
```

## License

MIT
