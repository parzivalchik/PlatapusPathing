package com.platapus.pathing.geometry;

import com.platapus.pathing.units.DistanceUnit;

/**
 * A robot pose: a position plus a heading.
 *
 * <p><b>Frame.</b> Poses are absolute <i>field</i> coordinates: the origin is the
 * centre of the field, +X and +Y lie in the floor plane, and heading is
 * CCW-positive radians from +X. See {@link FieldCoordinates} for the full
 * convention and the one axis question you have to settle on a real field.
 *
 * <p>So a pose means the same thing whichever tile the robot started on, and
 * {@code new Pose2d(24, 0, 0)} is a fixed spot two feet from centre. The cost is
 * that the library must be told where the robot begins -- absolute coordinates
 * cannot be inferred from encoders. Give your localizer a {@code startPose}.
 */
public final class Pose2d {

    public static final Pose2d ZERO = new Pose2d(0, 0, 0);

    /** Position in field coordinates, canonical inches. */
    public final Vector2d position;
    /** Heading, radians CCW from +X. */
    public final double heading;

    public Pose2d(Vector2d position, double heading) {
        this.position = position;
        this.heading = heading;
    }

    public Pose2d(double x, double y, double heading) {
        this(new Vector2d(x, y), heading);
    }

    /** Builds a pose from a position given in {@code unit}. Heading stays radians. */
    public static Pose2d of(double x, double y, double headingRad, DistanceUnit unit) {
        return new Pose2d(unit.toInches(x), unit.toInches(y), headingRad);
    }

    public double getX() {
        return position.x;
    }

    public double getY() {
        return position.y;
    }

    /** X in the requested unit. */
    public double getX(DistanceUnit unit) {
        return unit.fromInches(position.x);
    }

    /** Y in the requested unit. */
    public double getY(DistanceUnit unit) {
        return unit.fromInches(position.y);
    }

    public double getHeading() {
        return heading;
    }

    public Pose2d withHeading(double newHeading) {
        return new Pose2d(position, newHeading);
    }

    public Pose2d withPosition(Vector2d newPosition) {
        return new Pose2d(newPosition, heading);
    }

    /**
     * Composes {@code other} onto this pose, treating {@code other} as a pose
     * expressed in this pose's own frame.
     *
     * <p>Paths are already in field coordinates, so this is no longer needed to
     * place them. It is still the right tool for offsets that are genuinely
     * robot-relative -- where a mechanism sits on the chassis, or "two feet
     * further along whatever way I am currently facing".
     */
    public Pose2d transformBy(Pose2d other) {
        return new Pose2d(position.plus(other.position.rotated(heading)),
                Angles.normalize(heading + other.heading));
    }

    /**
     * The inverse of {@link #transformBy}: expresses this pose in {@code
     * reference}'s frame.
     */
    public Pose2d relativeTo(Pose2d reference) {
        Vector2d d = position.minus(reference.position).rotated(-reference.heading);
        return new Pose2d(d, Angles.normalize(heading - reference.heading));
    }

    /**
     * The translational error from this pose to {@code target}, rotated into this
     * pose's own (robot) frame. +x is forward, +y is left. This is the form the
     * follower feeds to its controllers.
     */
    public Vector2d errorTo(Vector2d target) {
        return target.minus(position).rotated(-heading);
    }

    /** Shortest signed rotation from this heading to {@code targetHeading}. */
    public double headingErrorTo(double targetHeading) {
        return Angles.normalize(targetHeading - heading);
    }

    public boolean epsilonEquals(Pose2d o, double posEps, double headingEps) {
        return position.epsilonEquals(o.position, posEps)
                && Math.abs(Angles.normalize(heading - o.heading)) < headingEps;
    }

    @Override
    public String toString() {
        return String.format("Pose2d(%.3f, %.3f, %.1f deg)",
                position.x, position.y, Math.toDegrees(heading));
    }
}
