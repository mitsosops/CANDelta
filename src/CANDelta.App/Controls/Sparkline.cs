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

        var pen = new Pen(Stroke, StrokeThickness);

        for (int i = 1; i < points.Count; i++)
        {
            context.DrawLine(pen, points[i - 1], points[i]);
        }
    }
}
