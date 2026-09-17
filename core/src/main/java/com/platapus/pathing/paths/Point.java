package com.platapus.pathing.paths;

import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.units.DistanceUnit;

/**
 * A control point for a Bezier path, in field coordinates.
 *
 * <p>{@code new Point(24, 0)} is a fixed spot on the field, two feet from centre
 * along +X -- not an offset from wherever the robot happens to start. See {@link
 * com.platapus.pathing.geometry.FieldCoordinates}.
 *
 * <p>The bare {@code (x, y)} constructor takes inches. To work in another unit,
 * use {@link #of(double, double, DistanceUnit)} -- the value is converted once,
 * here, and everything downstream is canonical.
 */
public final class Point {

    private final Vector2d vector;

    /** A point in field-coordinate inches. */
    public Point(double x, double y) {
        this.vector = new Vector2d(x, y);
    }

    public Point(Vector2d vector) {
        this.vector = vector;
    }

    /** A point in the given unit. */
    public static Point of(double x, double y, DistanceUnit unit) {
        return new Point(unit.toInches(x), unit.toInches(y));
    }

    public Vector2d toVector() {
        return vector;
    }

    public double getX() {
        return vector.x;
    }

    public double getY() {
        return vector.y;
    }

    @Override
    public String toString() {
        return "Point" + vector;
    }
}
