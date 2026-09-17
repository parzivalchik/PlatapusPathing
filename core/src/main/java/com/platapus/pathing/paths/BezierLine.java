package com.platapus.pathing.paths;

/**
 * A straight segment between two points -- a degree-1 Bezier.
 *
 * <p>Kept as its own type because it is by far the most common segment and
 * because reading {@code new BezierLine(a, b)} in an auto is clearer than a
 * two-point {@code BezierCurve}.
 */
public final class BezierLine extends BezierPath {

    public BezierLine(Point start, Point end) {
        super(start, end);
    }
}
