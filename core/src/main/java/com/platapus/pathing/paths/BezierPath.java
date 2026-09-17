package com.platapus.pathing.paths;

import com.platapus.pathing.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * A Bezier curve of arbitrary degree, evaluated with de Casteljau's algorithm.
 *
 * <p>{@link BezierLine} and {@link BezierCurve} are thin subclasses that fix the
 * degree; all the actual math lives here.
 *
 * <p>Arc length is handled by building a lookup table of cumulative distance at
 * construction time. That costs a few hundred microseconds once, and makes
 * {@link #arcLengthAt} and {@link #tAtArcLength} constant-time afterwards -- which
 * matters, because the follower calls them every control loop.
 */
public class BezierPath implements Path {

    /** Samples in the arc-length table. 200 holds error well under 0.01" for
     *  field-sized curves while keeping construction cheap enough to run in an
     *  OpMode's init block. */
    private static final int ARC_SAMPLES = 200;

    private final List<Vector2d> controlPoints;
    private final List<Vector2d> firstDerivativePoints;
    private final List<Vector2d> secondDerivativePoints;

    /** cumulativeLength[i] = arc length from t=0 to t=i/ARC_SAMPLES. */
    private final double[] cumulativeLength;
    private final double totalLength;

    public BezierPath(List<Point> points) {
        if (points.size() < 2) {
            throw new IllegalArgumentException(
                    "A Bezier path needs at least 2 control points, got " + points.size());
        }

        List<Vector2d> cps = new ArrayList<>(points.size());
        for (Point p : points) {
            cps.add(p.toVector());
        }
        this.controlPoints = Collections.unmodifiableList(cps);
        this.firstDerivativePoints = derivativeControlPoints(cps);
        this.secondDerivativePoints = derivativeControlPoints(firstDerivativePoints);

        this.cumulativeLength = new double[ARC_SAMPLES + 1];
        double running = 0.0;
        Vector2d previous = deCasteljau(controlPoints, 0.0);
        cumulativeLength[0] = 0.0;
        for (int i = 1; i <= ARC_SAMPLES; i++) {
            Vector2d current = deCasteljau(controlPoints, (double) i / ARC_SAMPLES);
            running += current.distanceTo(previous);
            cumulativeLength[i] = running;
            previous = current;
        }
        this.totalLength = running;
    }

    public BezierPath(Point... points) {
        this(Arrays.asList(points));
    }

    /**
     * The control points of the derivative curve: for a degree-n Bezier with
     * control points P, the derivative is a degree-(n-1) Bezier with control
     * points n * (P[i+1] - P[i]).
     */
    private static List<Vector2d> derivativeControlPoints(List<Vector2d> points) {
        int n = points.size() - 1;
        if (n < 1) {
            return Collections.singletonList(Vector2d.ZERO);
        }
        List<Vector2d> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            out.add(points.get(i + 1).minus(points.get(i)).times(n));
        }
        return Collections.unmodifiableList(out);
    }

    /** Evaluates a Bezier with the given control points at {@code t}. */
    private static Vector2d deCasteljau(List<Vector2d> points, double t) {
        if (points.size() == 1) {
            return points.get(0);
        }
        Vector2d[] scratch = points.toArray(new Vector2d[0]);
        for (int level = scratch.length - 1; level > 0; level--) {
            for (int i = 0; i < level; i++) {
                scratch[i] = scratch[i].times(1.0 - t).plus(scratch[i + 1].times(t));
            }
        }
        return scratch[0];
    }

    /** The control points defining this curve, in order. Never empty. */
    public List<Vector2d> getControlPoints() {
        return controlPoints;
    }

    /** The degree of this curve: 1 for a line, 2 for a quadratic, and so on. */
    public int degree() {
        return controlPoints.size() - 1;
    }

    @Override
    public Vector2d getPoint(double t) {
        return deCasteljau(controlPoints, clamp01(t));
    }

    @Override
    public Vector2d getDerivative(double t) {
        return deCasteljau(firstDerivativePoints, clamp01(t));
    }

    @Override
    public Vector2d getSecondDerivative(double t) {
        return deCasteljau(secondDerivativePoints, clamp01(t));
    }

    @Override
    public double length() {
        return totalLength;
    }

    @Override
    public double arcLengthAt(double t) {
        double clamped = clamp01(t);
        double scaled = clamped * ARC_SAMPLES;
        int index = (int) Math.floor(scaled);
        if (index >= ARC_SAMPLES) {
            return totalLength;
        }
        double frac = scaled - index;
        return cumulativeLength[index]
                + (cumulativeLength[index + 1] - cumulativeLength[index]) * frac;
    }

    @Override
    public double tAtArcLength(double s) {
        if (s <= 0 || totalLength < 1e-9) {
            return 0.0;
        }
        if (s >= totalLength) {
            return 1.0;
        }

        // Binary search the cumulative table, then linearly interpolate within
        // the bracketing sample interval.
        int low = 0;
        int high = ARC_SAMPLES;
        while (high - low > 1) {
            int mid = (low + high) >>> 1;
            if (cumulativeLength[mid] <= s) {
                low = mid;
            } else {
                high = mid;
            }
        }
        double segment = cumulativeLength[high] - cumulativeLength[low];
        double frac = segment < 1e-12 ? 0.0 : (s - cumulativeLength[low]) / segment;
        return (low + frac) / ARC_SAMPLES;
    }

    @Override
    public double getClosestT(Vector2d query, double tGuess) {
        // Coarse scan for a global bracket, so a bad guess cannot strand the
        // search in the wrong local minimum...
        double bestT = 0.0;
        double bestDistance = Double.MAX_VALUE;
        for (int i = 0; i <= ARC_SAMPLES; i++) {
            double t = (double) i / ARC_SAMPLES;
            double d = getPoint(t).distanceTo(query);
            if (d < bestDistance) {
                bestDistance = d;
                bestT = t;
            }
        }

        // ...but prefer the neighbourhood of the caller's guess when it is
        // genuinely competitive. On a path that crosses itself, both lobes are
        // near-equidistant and the guess is what breaks the tie correctly.
        double guess = clamp01(tGuess);
        double guessDistance = getPoint(guess).distanceTo(query);
        if (guessDistance <= bestDistance + 1e-6) {
            bestT = guess;
        }

        // Golden-section refinement inside one sample interval.
        double step = 1.0 / ARC_SAMPLES;
        double low = clamp01(bestT - step);
        double high = clamp01(bestT + step);
        for (int i = 0; i < 40 && high - low > 1e-9; i++) {
            double m1 = low + (high - low) / 3.0;
            double m2 = high - (high - low) / 3.0;
            if (getPoint(m1).distanceTo(query) < getPoint(m2).distanceTo(query)) {
                high = m2;
            } else {
                low = m1;
            }
        }
        return (low + high) / 2.0;
    }

    private static double clamp01(double t) {
        return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    }

    @Override
    public String toString() {
        return getClass().getSimpleName() + "(degree=" + degree()
                + ", length=" + String.format("%.2f", totalLength) + "\")";
    }
}
