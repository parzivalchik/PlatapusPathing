package com.platapus.pathing.paths;

import com.platapus.pathing.geometry.Vector2d;

/**
 * A parametric curve in the plane. Pure geometry -- a {@code Path} carries no
 * heading, no velocity, and no notion of time. Heading is layered on top by a
 * {@link HeadingInterpolator} inside a {@link PathSegment}.
 *
 * <p>The parameter {@code t} runs 0 to 1 and is <b>not</b> proportional to
 * distance: a Bezier moves faster through the middle of its parameter range than
 * near its ends. Anything that needs even spacing -- the motion profile, the
 * follower -- must go through the arc-length methods rather than stepping {@code t}.
 */
public interface Path {

    /** Position at parameter {@code t} in [0, 1]. */
    Vector2d getPoint(double t);

    /** First derivative with respect to {@code t}. Direction of travel. */
    Vector2d getDerivative(double t);

    /** Second derivative with respect to {@code t}. */
    Vector2d getSecondDerivative(double t);

    /** Total arc length, inches. */
    double length();

    /** Arc length from the start of the path to parameter {@code t}, inches. */
    double arcLengthAt(double t);

    /** The parameter {@code t} at which arc length equals {@code s} inches. */
    double tAtArcLength(double s);

    /**
     * The parameter of the point on this path closest to {@code query}.
     *
     * @param tGuess where to start searching, normally the previous loop's
     *               result. Passing a good guess keeps the follower's projection
     *               local, so a path that doubles back on itself does not snap
     *               to the wrong lobe.
     */
    double getClosestT(Vector2d query, double tGuess);

    /** Unit tangent at {@code t}, or {@link Vector2d#ZERO} at a cusp. */
    default Vector2d getUnitTangent(double t) {
        return getDerivative(t).normalized();
    }

    /**
     * Signed curvature at {@code t}, 1/inches. Positive curves left. Returns 0
     * where the derivative vanishes.
     */
    default double getCurvature(double t) {
        Vector2d d1 = getDerivative(t);
        Vector2d d2 = getSecondDerivative(t);
        double speed = d1.norm();
        if (speed < 1e-9) {
            return 0.0;
        }
        return d1.cross(d2) / (speed * speed * speed);
    }

    /** The path's start point. */
    default Vector2d getStartPoint() {
        return getPoint(0.0);
    }

    /** The path's end point. */
    default Vector2d getEndPoint() {
        return getPoint(1.0);
    }
}
