package com.platapus.pathing.paths;

import com.platapus.pathing.geometry.Angles;

/**
 * Decides what the robot should be facing at each point along a path segment.
 *
 * <p>Kept separate from {@link Path} so that geometry and heading stay
 * independent: the same curve can be driven nose-first, held at a constant
 * heading for a shooter, or swept through a turn, without ever touching the
 * curve itself.
 */
public interface HeadingInterpolator {

    /**
     * The desired heading, radians CCW from field +X, at parameter {@code t}
     * along {@code path}.
     */
    double heading(Path path, double t);

    /** Holds one heading for the whole segment. */
    static HeadingInterpolator constant(double headingRad) {
        return (path, t) -> Angles.normalize(headingRad);
    }

    /**
     * Sweeps from {@code startRad} to {@code endRad} linearly in {@code t},
     * turning the short way around.
     *
     * <p>Note this is linear in the curve parameter, not in arc length, so on a
     * strongly non-uniform Bezier the turn is slightly front- or back-loaded.
     * That is the same behaviour teams are used to from Pedro Pathing.
     */
    static HeadingInterpolator linear(double startRad, double endRad) {
        return (path, t) -> Angles.lerpShortest(startRad, endRad, t);
    }

    /** Sweeps from {@code startRad} to {@code endRad} the long way around. */
    static HeadingInterpolator linearReversed(double startRad, double endRad) {
        return (path, t) -> Angles.lerpLongest(startRad, endRad, t);
    }

    /**
     * Points the robot along the direction of travel. The default when a segment
     * says nothing about heading.
     *
     * <p>At a cusp the tangent vanishes and there is no meaningful direction; the
     * heading from just before the cusp is reused rather than snapping to zero.
     */
    static HeadingInterpolator tangent() {
        return (path, t) -> {
            double angle = path.getDerivative(t).angle();
            if (path.getDerivative(t).norm() < 1e-9) {
                double back = Math.max(0.0, t - 1e-3);
                angle = path.getDerivative(back).angle();
            }
            return Angles.normalize(angle);
        };
    }

    /** Points the robot backwards along the direction of travel. */
    static HeadingInterpolator reverseTangent() {
        return (path, t) -> Angles.normalize(tangent().heading(path, t) + Math.PI);
    }

    /** Holds a fixed offset from the tangent -- useful for side intakes. */
    static HeadingInterpolator tangentOffset(double offsetRad) {
        return (path, t) -> Angles.normalize(tangent().heading(path, t) + offsetRad);
    }
}
