using System.Text;
using System.Drawing;
using System.Globalization;
using System.Drawing.Drawing2D;

namespace CsPotrace;

public partial class Potrace
{
    /// <summary>
    /// Exports a figure,  created by Potrace from a Bitmap to a svg-formatted string 
    ///It should be SVG-formattted

    /// </summary>
    /// <param name="Fig">Arraylist, which contains vectorinformations about the Curves</param>
    /// <param name="Width">Width of the Bitmap</param>
    /// <param name="Height">Height of the Bitmap</param>
    /// <returns></returns>
    public static Bitmap ExportBitmap(List<List<Curve[]>> Fig, int Width, int Height)
    {
        var bitmap = new Bitmap(Width, Height);
        using var g = Graphics.FromImage(bitmap);
        g.FillRectangle(new SolidBrush(Color.White), 0, 0, Width, Height);
        var gp = new GraphicsPath();
        for (int i = 0; i < Fig.Count; i++)
        {
            var CurveArray = Fig[i];
            GraphicsPath Contour = null;
            GraphicsPath Hole = null;
            GraphicsPath Current = null;
            for (int j = 0; j < CurveArray.Count; j++)
            {
                if (j == 0)
                {
                    Contour = new GraphicsPath();
                    Current = Contour;
                }
                else
                {

                    Hole = new GraphicsPath();
                    Current = Hole;
                }
                var Curves = CurveArray[j];
                float factor = 1;
                for (int k = 0; k < Curves.Length; k++)
                {
                    if (Curves[k].Kind == CurveKind.Bezier)
                        Current.AddBezier((float)Curves[k].A.X * factor, (float)Curves[k].A.Y * factor, (float)Curves[k].ControlPointA.X * factor, (float)Curves[k].ControlPointA.Y * factor,
                                    (float)Curves[k].ControlPointB.X * factor, (float)Curves[k].ControlPointB.Y * factor, (float)Curves[k].B.X * factor, (float)Curves[k].B.Y * factor);
                    else
                        Current.AddLine((float)Curves[k].A.X * factor, (float)Curves[k].A.Y * factor, (float)Curves[k].B.X * factor, (float)Curves[k].B.Y * factor);

                }
                if (j > 0) Contour.AddPath(Hole, false);
            }
            gp.AddPath(Contour, false);
        }

        g.FillPath(Brushes.Black, gp);

        return bitmap;
    }
    /// <summary>
    /// Exports a figure, created by Potrace from a Bitmap to a svg-formatted string
    /// </summary>
    /// <param name="Figure">Arraylist, which contains vectorinformations about the Curves</param>
    /// <param name="Width">Width of the exportd cvg-File</param>
    /// <param name="Height">Height of the exportd cvg-File</param>
    /// <returns></returns>
    public static string ExportSvg(List<List<Curve[]>> Figure, int Width, int Height)
    {
        var svg = new StringBuilder();
        var head = $@"<?xml version='1.0' standalone='no'?>
<!DOCTYPE svg PUBLIC '-//W3C//DTD SVG 20010904//EN' 
'http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd'>
<svg version='1.0' xmlns='http://www.w3.org/2000/svg' preserveAspectRatio='xMidYMid meet'
width='{Width}' height='{Height}'  viewBox='0 0 {Width} {Height}'>
<g>";
        svg.AppendLine(head);
        foreach (var Path in Figure)
        {
            var isContour = true;
            svg.Append("<path d='");
            for (int i = 0; i < Path.Count; i++)
            {
                var Curves = Path[i];
                var CurvePath = isContour ? GetContourPath(Curves) : GetHolePath(Curves);
                if (i == Path.Count - 1)
                {
                    svg.Append(CurvePath);
                }
                else
                {
                    svg.AppendLine(CurvePath);
                }
                isContour = false;
            }
            svg.AppendLine("' />");
        }

        var foot = @"</g>
</svg>";
        svg.AppendLine(foot);
        svg.Replace("'", "\"");
        return svg.ToString();
    }
    static CultureInfo enUsCulture = CultureInfo.GetCultureInfo("en-US");

    private static string GetContourPath(Curve[] Curves)
    {
        var path = new StringBuilder();
        for (int i = 0; i < Curves.Length; i++)
        {
            var Curve = Curves[i];
            if (i == 0)
            {
                path.AppendLine($"M{Curve.A.X.ToString("0.0", enUsCulture)} {Curve.A.Y.ToString("0.0", enUsCulture)}");
            }

            if (Curve.Kind == CurveKind.Bezier)
            {
                path.Append("C" + Curve.ControlPointA.X.ToString("0.0", enUsCulture) + " " + Curve.ControlPointA.Y.ToString("0.0", enUsCulture) + " " +
                                Curve.ControlPointB.X.ToString("0.0", enUsCulture) + " " + Curve.ControlPointB.Y.ToString("0.0", enUsCulture) + " " +
                                Curve.B.X.ToString("0.0", enUsCulture) + " " + Curve.B.Y.ToString("0.0", enUsCulture));
            }
            if (Curve.Kind == CurveKind.Line)
            {
                path.Append("L" + Curve.B.X.ToString("0.0", enUsCulture) + " " + Curve.B.Y.ToString("0.0", enUsCulture));
            }
            if (i == Curves.Length - 1)
            {
                path.Append("Z");
            }
            else
            {
                path.AppendLine("");
            }
        }
        return path.ToString();
    }

    private static string GetHolePath(Curve[] Curves)
    {
        var path = new StringBuilder();

        for (int i = Curves.Length - 1; i >= 0; i--)
        {
            var Curve = Curves[i];
            if (i == Curves.Length - 1)
            {
                path.AppendLine($"M{Curve.B.X.ToString("0.0", enUsCulture)} {Curve.B.Y.ToString("0.0", enUsCulture)}");
            }

            if (Curve.Kind == CurveKind.Bezier)
            {
                path.Append($"C{Curve.ControlPointB.X.ToString("0.0", enUsCulture)} {Curve.ControlPointB.Y.ToString("0.0", enUsCulture)} {Curve.ControlPointA.X.ToString("0.0", enUsCulture)} {Curve.ControlPointA.Y.ToString("0.0", enUsCulture)} {Curve.A.X.ToString("0.0", enUsCulture)} {Curve.A.Y.ToString("0.0", enUsCulture)}");
            }
            if (Curve.Kind == CurveKind.Line)
            {
                path.Append("L" + Curve.B.X.ToString("0.0", enUsCulture) + " " + Curve.B.Y.ToString("0.0", enUsCulture));
            }
            if (i == 0)
            {
                path.Append("Z");
            }
            else
            {
                path.AppendLine();
            }
        }
        return path.ToString();
    }
}
