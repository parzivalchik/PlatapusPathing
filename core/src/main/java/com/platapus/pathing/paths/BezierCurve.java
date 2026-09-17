package com.platapus.pathing.paths;

import java.util.Arrays;
import java.util.List;

/**
 * A Bezier curve of degree 2 or higher: a start point, one or more control
 * points, and an end point.
 *
 * <p>The curve passes through its first and last points and is only pulled
 * toward the intermediate ones.
 */
public final class BezierCurve extends BezierPath {

    public BezierCurve(Point... points) {
        super(requireCurve(Arrays.asList(points)));
    }

    public BezierCurve(List<Point> points) {
        super(requireCurve(points));
    }

    private static List<Point> requireCurve(List<Point> points) {
        if (points.size() < 3) {
            throw new IllegalArgumentException(
                    "BezierCurve needs at least 3 control points (got " + points.size()
                            + "); use BezierLine for a straight segment.");
        }
        return points;
    }
}
