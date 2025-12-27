using CANDelta.Core.Analysis;
using CANDelta.Core.Models;
using CANDelta.Core.Protocol;
using Xunit;

namespace CANDelta.Core.Tests;

public class DeltaAnalyzerTests
{
    private readonly DeltaAnalyzer _analyzer = new();

    [Fact]
    public void Analyze_FindsUniqueTestFrames()
    {
        // Arrange
        var control = CreateBehaviour("Control", new[] { 0x100u, 0x200u, 0x300u });
        var test = CreateBehaviour("Test", new[] { 0x100u, 0x200u, 0x400u }); // 0x400 is unique

        // Act
        var result = _analyzer.Analyze(control, test, noiseThreshold: 0);

        // Assert
        Assert.Single(result.UniqueToTest);
        Assert.Equal(0x400u, result.UniqueToTest[0].Id);
    }

    [Fact]
    public void Analyze_FindsUniqueControlFrames()
    {
        // Arrange
        var control = CreateBehaviour("Control", new[] { 0x100u, 0x200u, 0x300u });
        var test = CreateBehaviour("Test", new[] { 0x100u, 0x200u }); // 0x300 missing

        // Act
        var result = _analyzer.Analyze(control, test, noiseThreshold: 0);

        // Assert
        Assert.Single(result.UniqueToControl);
        Assert.Equal(0x300u, result.UniqueToControl[0].Id);
    }

    [Fact]
    public void Analyze_FiltersNoiseBelowThreshold()
    {
        // Arrange: Frame 0x999 only in 1 of 3 traces (33%)
        var control = CreateBehaviour("Control", new[] { 0x100u });
        var test = new Behaviour { Name = "Test" };
        test.Traces.Add(CreateTrace(new[] { 0x100u, 0x999u }));
        test.Traces.Add(CreateTrace(new[] { 0x100u }));
        test.Traces.Add(CreateTrace(new[] { 0x100u }));

        // Act: 50% threshold should filter out 0x999
        var result = _analyzer.Analyze(control, test, noiseThreshold: 0.5);

        // Assert
        Assert.Empty(result.UniqueToTest);
    }

    [Fact]
    public void Analyze_IncludesFramesAboveThreshold()
    {
        // Arrange: Frame 0x999 in 2 of 3 traces (67%)
        var control = CreateBehaviour("Control", new[] { 0x100u });
        var test = new Behaviour { Name = "Test" };
        test.Traces.Add(CreateTrace(new[] { 0x100u, 0x999u }));
        test.Traces.Add(CreateTrace(new[] { 0x100u, 0x999u }));
        test.Traces.Add(CreateTrace(new[] { 0x100u }));

        // Act: 50% threshold should include 0x999
        var result = _analyzer.Analyze(control, test, noiseThreshold: 0.5);

        // Assert
        Assert.Single(result.UniqueToTest);
        Assert.Equal(0x999u, result.UniqueToTest[0].Id);
    }

    private static Behaviour CreateBehaviour(string name, uint[] canIds)
    {
        var behaviour = new Behaviour { Name = name };
        behaviour.Traces.Add(CreateTrace(canIds));
        return behaviour;
    }

    private static Trace CreateTrace(uint[] canIds)
    {
        var trace = new Trace();
        foreach (var id in canIds)
        {
            trace.Frames.Add(new CanFrame
            {
                Id = id,
                Dlc = 8,
                Data = new byte[8],
                TimestampUs = 0
            });
        }
        return trace;
    }
}
