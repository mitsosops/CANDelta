using CANDelta.Core.Models;
using CANDelta.Core.Protocol;

namespace CANDelta.Core.Analysis;

/// <summary>
/// Analyzes CAN traces to find deltas between control and test behaviours.
/// </summary>
public class DeltaAnalyzer
{
    /// <summary>
    /// Default threshold: frame must appear in at least 50% of traces to be considered signal (not noise).
    /// </summary>
    public const double DefaultNoiseThreshold = 0.5;

    /// <summary>
    /// Analyze the delta between a control behaviour and a test behaviour.
    /// </summary>
    /// <param name="control">The baseline/control behaviour.</param>
    /// <param name="test">The test behaviour to compare.</param>
    /// <param name="noiseThreshold">Minimum occurrence percentage (0-1) for a frame to be considered signal.</param>
    /// <param name="matchDataPattern">If true, compare full data patterns. If false, only compare CAN IDs.</param>
    /// <returns>The delta analysis result.</returns>
    public DeltaResult Analyze(Behaviour control, Behaviour test, double noiseThreshold = DefaultNoiseThreshold, bool matchDataPattern = false)
    {
        var controlSignatures = BuildSignatureMap(control.Traces, noiseThreshold, matchDataPattern);
        var testSignatures = BuildSignatureMap(test.Traces, noiseThreshold, matchDataPattern);

        var uniqueToTest = new List<DeltaFrame>();
        var uniqueToControl = new List<DeltaFrame>();
        var modified = new List<DeltaFrame>();

        // Find frames unique to test
        foreach (var (sig, info) in testSignatures)
        {
            if (!controlSignatures.ContainsKey(info.IdSignature))
            {
                uniqueToTest.Add(CreateDeltaFrame(info, test.Traces.Count));
            }
            else if (matchDataPattern && !controlSignatures.ContainsKey(sig))
            {
                modified.Add(CreateDeltaFrame(info, test.Traces.Count));
            }
        }

        // Find frames unique to control
        foreach (var (sig, info) in controlSignatures)
        {
            if (!testSignatures.ContainsKey(info.IdSignature))
            {
                uniqueToControl.Add(CreateDeltaFrame(info, control.Traces.Count));
            }
        }

        // Sort by occurrence count (most frequent first)
        uniqueToTest.Sort((a, b) => b.OccurrenceCount.CompareTo(a.OccurrenceCount));
        uniqueToControl.Sort((a, b) => b.OccurrenceCount.CompareTo(a.OccurrenceCount));
        modified.Sort((a, b) => b.OccurrenceCount.CompareTo(a.OccurrenceCount));

        return new DeltaResult
        {
            ControlBehaviour = control,
            TestBehaviour = test,
            UniqueToTest = uniqueToTest,
            UniqueToControl = uniqueToControl,
            ModifiedFrames = modified,
            NoiseThreshold = noiseThreshold
        };
    }

    private static Dictionary<string, FrameInfo> BuildSignatureMap(
        List<Trace> traces, double noiseThreshold, bool matchDataPattern)
    {
        if (traces.Count == 0) return new Dictionary<string, FrameInfo>();

        var signatureCounts = new Dictionary<string, FrameInfo>();

        foreach (var trace in traces)
        {
            var seenInTrace = new HashSet<string>();

            foreach (var frame in trace.Frames)
            {
                var sig = matchDataPattern ? frame.GetSignature() : frame.GetIdSignature();

                if (!seenInTrace.Contains(sig))
                {
                    seenInTrace.Add(sig);

                    if (!signatureCounts.TryGetValue(sig, out var info))
                    {
                        info = new FrameInfo
                        {
                            Id = frame.Id,
                            IsExtended = frame.IsExtended,
                            IdSignature = frame.GetIdSignature()
                        };
                        signatureCounts[sig] = info;
                    }

                    info.TraceCount++;
                    info.TotalCount++;
                    info.Examples.Add(frame);

                    if (frame.Data != null && frame.Data.Length > 0)
                    {
                        var dataKey = Convert.ToHexString(frame.Data);
                        if (!info.DataPatternsSet.Contains(dataKey))
                        {
                            info.DataPatternsSet.Add(dataKey);
                            info.DataPatterns.Add((byte[])frame.Data.Clone());
                        }
                    }
                }
                else
                {
                    if (signatureCounts.TryGetValue(sig, out var info))
                    {
                        info.TotalCount++;
                    }
                }
            }
        }

        // Filter by noise threshold
        var minTraces = (int)Math.Ceiling(traces.Count * noiseThreshold);
        var result = new Dictionary<string, FrameInfo>();

        foreach (var (sig, info) in signatureCounts)
        {
            if (info.TraceCount >= minTraces)
            {
                result[sig] = info;
            }
        }

        return result;
    }

    private static DeltaFrame CreateDeltaFrame(FrameInfo info, int totalTraces)
    {
        return new DeltaFrame
        {
            Id = info.Id,
            IsExtended = info.IsExtended,
            OccurrenceCount = info.TotalCount,
            OccurrencePercentage = totalTraces > 0 ? (double)info.TraceCount / totalTraces : 0,
            Examples = info.Examples.Take(10).ToList(),
            DataPatterns = info.DataPatterns,
            Confidence = totalTraces > 0 ? (double)info.TraceCount / totalTraces : 0
        };
    }

    private class FrameInfo
    {
        public uint Id { get; init; }
        public bool IsExtended { get; init; }
        public string IdSignature { get; init; } = "";
        public int TraceCount { get; set; }
        public int TotalCount { get; set; }
        public List<CanFrame> Examples { get; } = new();
        public HashSet<string> DataPatternsSet { get; } = new();
        public List<byte[]> DataPatterns { get; } = new();
    }
}
