using CANDelta.Core.Protocol;

namespace CANDelta.Core.Tests;

public class ProtocolParserTests
{
    [Fact]
    public void SerializeCommand_CreatesValidPacket()
    {
        // Act
        var packet = ProtocolParser.SerializeCommand(CommandOpcode.Ping);

        // Assert
        Assert.Equal(4, packet.Length);
        Assert.Equal(0x02, packet[0]); // STX
        Assert.Equal((byte)CommandOpcode.Ping, packet[1]);
        Assert.Equal(0, packet[2]); // param length
        Assert.Equal(0x03, packet[3]); // ETX
    }

    [Fact]
    public void CreateSetSpeedCommand_SerializesCorrectly()
    {
        // Act
        var packet = ProtocolParser.CreateSetSpeedCommand(CanSpeed.Speed500Kbps);

        // Assert
        Assert.Equal(8, packet.Length); // STX + opcode + len + 4 bytes + ETX
        Assert.Equal(0x02, packet[0]);
        Assert.Equal((byte)CommandOpcode.SetSpeed, packet[1]);
        Assert.Equal(4, packet[2]); // 4 bytes for speed

        // 500000 = 0x0007A120 in little-endian
        Assert.Equal(0x20, packet[3]);
        Assert.Equal(0xA1, packet[4]);
        Assert.Equal(0x07, packet[5]);
        Assert.Equal(0x00, packet[6]);
        Assert.Equal(0x03, packet[7]);
    }

    [Fact]
    public void Parser_RaisesFrameReceived_OnValidFrame()
    {
        // Arrange
        var parser = new ProtocolParser();
        CanFrame? receivedFrame = null;
        parser.FrameReceived += f => receivedFrame = f;

        // Build a CAN frame packet:
        // STX + timestamp(8) + id(4) + flags(1) + dlc(1) + data(dlc) + ETX
        var packet = new byte[]
        {
            0x02, // STX
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // timestamp = 0x0100000000000000
            0x23, 0x01, 0x00, 0x00, // id = 0x123
            0x00, // flags (standard, not RTR)
            0x02, // dlc = 2
            0xAB, 0xCD, // data
            0x03  // ETX
        };

        // Act
        parser.ProcessBytes(packet, 0, packet.Length);

        // Assert
        Assert.NotNull(receivedFrame);
        Assert.Equal(0x123u, receivedFrame.Value.Id);
        Assert.Equal(2, receivedFrame.Value.Dlc);
        Assert.Equal(new byte[] { 0xAB, 0xCD }, receivedFrame.Value.Data);
        Assert.False(receivedFrame.Value.IsExtended);
    }

    [Fact]
    public void Parser_RaisesResponseReceived_OnAck()
    {
        // Arrange
        var parser = new ProtocolParser();
        ResponseCode? receivedCode = null;
        parser.ResponseReceived += (code, data) => receivedCode = code;

        var packet = new byte[]
        {
            0x02, // STX
            0x80, // ACK
            0x00, // no data
            0x03  // ETX
        };

        // Act
        parser.ProcessBytes(packet, 0, packet.Length);

        // Assert
        Assert.Equal(ResponseCode.Ack, receivedCode);
    }

    [Fact]
    public void ParseVersionResponse_ParsesCorrectly()
    {
        // Arrange
        var data = new byte[] { 0x01, 0x00, 0x01, 0x00 }; // protocol v1, version 0.1.0

        // Act
        var version = ProtocolParser.ParseVersionResponse(data);

        // Assert
        Assert.NotNull(version);
        Assert.Equal(1, version.ProtocolVersion);
        Assert.Equal(0, version.Major);
        Assert.Equal(1, version.Minor);
        Assert.Equal(0, version.Patch);
    }
}
