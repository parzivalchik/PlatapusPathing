package com.platapus.pathing.geometry;

import com.platapus.pathing.units.DistanceUnit;

/**
 * The field coordinate frame every pose and path point in this library lives in.
 *
 * <h2>The frame</h2>
 * <ul>
 *   <li><b>Origin</b> at the centre of the field, on the floor.</li>
 *   <li><b>+X</b> and <b>+Y</b> in the floor plane, right-handed with +Z up.</li>
 *   <li><b>Heading</b> in radians, CCW-positive, measured from +X.</li>
 *   <li>The field is {@value #FIELD_SIZE_INCHES} inches square, so every
 *       coordinate lies in [-72, +72].</li>
 * </ul>
 *
 * <p>So {@code new Point(24, 0)} is a fixed spot on the field — two feet from
 * centre along +X — and it means the same thing whichever tile the robot starts
 * on. This is the same convention the FTC SDK's AprilTag support uses, which is
 * what lets a tag observation drop straight into the pose filter.
 *
 * <h2>The one thing you must confirm on the field</h2>
 * The origin and the handedness above are not negotiable. <b>Which physical wall
 * +X points at is a choice</b>, and it has to match how your AprilTag layout and
 * your heading sensor are set up. This library deliberately does not bake in a
 * season-specific wall mapping, because getting that silently wrong is expensive
 * and it is one line for you to pin down.
 *
 * <p>Check it once, on a real field, before trusting a long auto: place the robot
 * at a known spot facing a known wall, call {@link
 * com.platapus.pathing.localization.Localizer#getPose()}, and confirm the
 * signs are what you expect. If +X turns out to point at the opposite wall,
 * rotate your whole plan by 180 degrees rather than negating coordinates
 * one at a time.
 *
 * <h2>Starting pose</h2>
 * Because coordinates are absolute, the library has to be told where the robot
 * actually is when the auto begins — it cannot work that out from the encoders.
 * Give the localizer a {@code startPose}:
 *
 * <pre>{@code
 * OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
 *         .startPose(new Pose2d(-60, -36, 0))   // where the robot is placed
 *         .build();
 * }</pre>
 *
 * <p>Get that wrong and every path in the auto is offset by the same error,
 * which usually looks like the robot "driving the right shape in the wrong
 * place". It is the first thing to check when an auto is uniformly off.
 */
public final class FieldCoordinates {

    private FieldCoordinates() {
    }

    /** The field is 12 feet square. */
    public static final double FIELD_SIZE_INCHES = 144.0;

    /** Half the field, so the coordinate limit on each axis. */
    public static final double HALF_FIELD_INCHES = FIELD_SIZE_INCHES / 2.0;

    /** One field tile. */
    public static final double TILE_INCHES = 24.0;

    /** The centre of the field, facing +X. */
    public static final Pose2d ORIGIN = Pose2d.ZERO;

    /** The field size in the requested unit. */
    public static double fieldSize(DistanceUnit unit) {
        return unit.fromInches(FIELD_SIZE_INCHES);
    }

    /** Whether a point is inside the field walls. */
    public static boolean contains(Vector2d point) {
        return Math.abs(point.x) <= HALF_FIELD_INCHES
                && Math.abs(point.y) <= HALF_FIELD_INCHES;
    }

    /**
     * Whether a robot footprint centred at {@code pose} fits inside the walls.
     *
     * <p>Worth asserting on a planned path: a point can be legal while the robot
     * standing on it is not.
     */
    public static boolean contains(Pose2d pose, double lengthInches, double widthInches) {
        double hl = lengthInches / 2.0;
        double hw = widthInches / 2.0;
        for (double[] corner : new double[][]{{hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}}) {
            Vector2d p = new Vector2d(corner[0], corner[1])
                    .rotated(pose.heading).plus(pose.position);
            if (!contains(p)) {
                return false;
            }
        }
        return true;
    }

    /** Clamps a point inside the field walls. */
    public static Vector2d clampToField(Vector2d point) {
        return new Vector2d(
                Math.max(-HALF_FIELD_INCHES, Math.min(HALF_FIELD_INCHES, point.x)),
                Math.max(-HALF_FIELD_INCHES, Math.min(HALF_FIELD_INCHES, point.y)));
    }

    /**
     * The 180-degree rotation of a pose about the field centre.
     *
     * <p>The two alliance stations face each other, so a red auto is usually the
     * blue one turned around. This mirrors a whole plan in one step instead of
     * negating coordinates by hand — and it is also the fix if you discover +X
     * points at the opposite wall from what you assumed.
     */
    public static Pose2d rotated180(Pose2d pose) {
        return new Pose2d(-pose.position.x, -pose.position.y,
                Angles.normalize(pose.heading + Math.PI));
    }

    /** The 180-degree rotation of a point about the field centre. */
    public static Vector2d rotated180(Vector2d point) {
        return new Vector2d(-point.x, -point.y);
    }
}
