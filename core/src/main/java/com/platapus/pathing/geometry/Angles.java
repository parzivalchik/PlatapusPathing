package com.platapus.pathing.geometry;

/** Angle helpers. All angles are radians. */
public final class Angles {

    private Angles() {
    }

    private static final double TWO_PI = 2.0 * Math.PI;

    /** Wraps to (-pi, pi]. */
    public static double normalize(double angleRad) {
        double a = angleRad % TWO_PI;
        if (a > Math.PI) {
            a -= TWO_PI;
        } else if (a <= -Math.PI) {
            a += TWO_PI;
        }
        return a;
    }

    /** Wraps to [0, 2pi). */
    public static double normalizePositive(double angleRad) {
        double a = angleRad % TWO_PI;
        return a < 0 ? a + TWO_PI : a;
    }

    /**
     * Interpolates from {@code start} to {@code end} the short way around,
     * {@code t} in [0, 1]. Used by linear heading interpolation so a 350 deg ->
     * 10 deg turn goes 20 degrees forward, not 340 back.
     */
    public static double lerpShortest(double start, double end, double t) {
        return normalize(start + normalize(end - start) * t);
    }

    /**
     * Interpolates the long way around -- deliberately turning through the
     * reflex angle. Occasionally wanted when a mechanism or a cable would
     * otherwise sweep through a hazard.
     */
    public static double lerpLongest(double start, double end, double t) {
        double delta = normalize(end - start);
        double longWay = delta >= 0 ? delta - TWO_PI : delta + TWO_PI;
        return normalize(start + longWay * t);
    }
}
