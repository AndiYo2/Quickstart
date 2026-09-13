package org.firstinspires.ftc.teamcode.pedro.integration;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

/**
 * Heading interpolators that work on straight lines.
 *
 * Pedro 3.0.0's {@code Interpolator.linear}/{@code piecewise} use {@code Curve.pathCompletion(t)}, whose default
 * implementation returns the fraction <em>remaining</em> ({@code remainingDistance / length}). {@code BezierCurve}
 * overrides it correctly, {@code Line} does not, so on {@code Paths.line(...)} the stock linear heading runs
 * backwards (starts at the end heading). These helpers compute the completed fraction directly and are correct
 * on every curve type. Use them instead of {@code .linear(...)} on line paths.
 */
public final class Headings {
    private Headings() {}

    /** Fraction of arc length completed at parameter t, correct for lines and Béziers. */
    public static double completed(Curve curve, double t) {
        double len = curve.length();
        if (len <= 0) return 1.0;
        double c = 1.0 - curve.remainingDistance(t) / len;
        return c < 0 ? 0 : (c > 1 ? 1 : c);
    }

    /** Shortest-way angular interpolation from a to b (radians), frac in [0, 1]. */
    public static double lerp(double a, double b, double frac) {
        double start = Angle.normalize(a);
        double delta = Angle.normalize(b) - start;
        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;
        return Angle.normalize(start + delta * frac);
    }

    /** Constant-rate turn from start to end (radians) over the whole path, shortest direction. */
    public static Interpolator linear(double start, double end) {
        return (curve, t) -> lerp(start, end, completed(curve, t));
    }

    public static Interpolator linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    /**
     * Piecewise-linear heading keyframes. {@code fractions} are completed-arc-length fractions in ascending
     * order starting at 0 and ending at 1; {@code headings} (radians) are the headings at those fractions.
     * Between keyframes the heading turns the short way at constant rate; equal neighbours hold the heading.
     */
    public static Interpolator keyframes(double[] fractions, double[] headings) {
        if (fractions.length != headings.length || fractions.length < 2)
            throw new IllegalArgumentException("need >= 2 matching fractions/headings");
        return (curve, t) -> {
            double c = completed(curve, t);
            for (int i = 1; i < fractions.length; i++) {
                if (c <= fractions[i] || i == fractions.length - 1) {
                    double span = fractions[i] - fractions[i - 1];
                    double f = span <= 1e-9 ? 1.0 : (c - fractions[i - 1]) / span;
                    return lerp(headings[i - 1], headings[i], Math.max(0, Math.min(1, f)));
                }
            }
            return Angle.normalize(headings[headings.length - 1]);
        };
    }
}
