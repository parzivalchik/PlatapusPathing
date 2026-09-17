package com.platapus.pathing.geometry;

/** An immutable 2d vector, in canonical inches. */
public final class Vector2d {

    public static final Vector2d ZERO = new Vector2d(0, 0);

    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /** Unit vector at {@code angleRad} counter-clockwise from +X. */
    public static Vector2d fromAngle(double angleRad) {
        return new Vector2d(Math.cos(angleRad), Math.sin(angleRad));
    }

    public Vector2d plus(Vector2d o) {
        return new Vector2d(x + o.x, y + o.y);
    }

    public Vector2d minus(Vector2d o) {
        return new Vector2d(x - o.x, y - o.y);
    }

    public Vector2d times(double s) {
        return new Vector2d(x * s, y * s);
    }

    public Vector2d div(double s) {
        return new Vector2d(x / s, y / s);
    }

    public Vector2d unaryMinus() {
        return new Vector2d(-x, -y);
    }

    public double dot(Vector2d o) {
        return x * o.x + y * o.y;
    }

    /** 2d cross product (the z component of the 3d cross). */
    public double cross(Vector2d o) {
        return x * o.y - y * o.x;
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double normSquared() {
        return x * x + y * y;
    }

    public double distanceTo(Vector2d o) {
        return Math.hypot(x - o.x, y - o.y);
    }

    /** Direction in radians, CCW from +X, in (-pi, pi]. */
    public double angle() {
        return Math.atan2(y, x);
    }

    /**
     * Returns this vector scaled to length 1, or {@link #ZERO} if this vector is
     * degenerate. Callers relying on a direction must check for zero themselves;
     * silently returning ZERO is deliberate so that a zero-length path tangent
     * cannot produce NaN inside the follower.
     */
    public Vector2d normalized() {
        double n = norm();
        return n < 1e-12 ? ZERO : new Vector2d(x / n, y / n);
    }

    /** Rotates CCW by {@code angleRad}. */
    public Vector2d rotated(double angleRad) {
        double c = Math.cos(angleRad);
        double s = Math.sin(angleRad);
        return new Vector2d(x * c - y * s, x * s + y * c);
    }

    /** Projects this onto {@code o}. Returns ZERO if {@code o} is degenerate. */
    public Vector2d projectOnto(Vector2d o) {
        double d = o.normSquared();
        return d < 1e-12 ? ZERO : o.times(dot(o) / d);
    }

    public boolean epsilonEquals(Vector2d o, double eps) {
        return Math.abs(x - o.x) < eps && Math.abs(y - o.y) < eps;
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }
}
