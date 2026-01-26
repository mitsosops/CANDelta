using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace CANDelta.App.Controls;

/// <summary>
/// A simple sparkline control that renders a line graph from a Points collection.
/// </summary>
public class Sparkline : Control
{
    public static readonly StyledProperty<Points> PointsProperty =
        AvaloniaProperty.Register<Sparkline, Points>(nameof(Points), new Points());

    public static readonly StyledProperty<IBrush> StrokeProperty =
        AvaloniaProperty.Register<Sparkline, IBrush>(nameof(Stroke), Brushes.Gray);

    public static readonly StyledProperty<double> StrokeThicknessProperty =
        AvaloniaProperty.Register<Sparkline, double>(nameof(StrokeThickness), 1.0);

    // Cached pen to avoid allocations on every render
    private Pen? _cachedPen;
    private IBrush? _cachedStroke;
    private double _cachedStrokeThickness;

    public Points Points
    {
        get => GetValue(PointsProperty);
        set => SetValue(PointsProperty, value);
    }

    public IBrush Stroke
    {
        get => GetValue(StrokeProperty);
        set => SetValue(StrokeProperty, value);
    }

    public double StrokeThickness
    {
        get => GetValue(StrokeThicknessProperty);
        set => SetValue(StrokeThicknessProperty, value);
    }

    static Sparkline()
    {
        AffectsRender<Sparkline>(PointsProperty, StrokeProperty, StrokeThicknessProperty);
    }

    public override void Render(DrawingContext context)
    {
        base.Render(context);

        var points = Points;
        if (points == null || points.Count < 2)
            return;

        var stroke = Stroke;
        var strokeThickness = StrokeThickness;

        // Only create new pen if stroke or thickness changed
        if (_cachedPen == null || _cachedStroke != stroke || _cachedStrokeThickness != strokeThickness)
        {
            _cachedPen = new Pen(stroke, strokeThickness);
            _cachedStroke = stroke;
            _cachedStrokeThickness = strokeThickness;
        }

        for (int i = 1; i < points.Count; i++)
        {
            context.DrawLine(_cachedPen, points[i - 1], points[i]);
        }
    }
}
