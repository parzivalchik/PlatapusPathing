package com.platapus.pathing.paths;

import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Vector2d;

/**
 * A fully resolved point on a {@link PathChain}: where it is, which way the path
 * is heading there, which way the robot should face, and how sharply the path is
 * turning.
 *
 * <p>This is the single object the follower asks the path layer for each loop.
 */
public final class PathState {

    /** Index of the segment this state falls on. */
    public final int segmentIndex;
    /** Parameter within that segment, [0, 1]. */
    public final double t;
    /** Arc length from the start of the whole chain, inches. */
    public final double arcLength;
    /** Position in field coordinates, inches. */
    public final Vector2d point;
    /** Unit tangent -- the direction the path travels here. */
    public final Vector2d tangent;
    /** Desired robot heading here, radians CCW. */
    public final double heading;
    /** Signed curvature, 1/inches. Positive turns left. */
    public final double curvature;

    public PathState(int segmentIndex, double t, double arcLength, Vector2d point,
                     Vector2d tangent, double heading, double curvature) {
        this.segmentIndex = segmentIndex;
        this.t = t;
        this.arcLength = arcLength;
        this.point = point;
        this.tangent = tangent;
        this.heading = heading;
        this.curvature = curvature;
    }

    /** This state as a pose: the path point with the desired heading. */
    public Pose2d toPose() {
        return new Pose2d(point, heading);
    }

    @Override
    public String toString() {
        return String.format("PathState(seg=%d, t=%.3f, s=%.2f\", %s, %.1f deg)",
                segmentIndex, t, arcLength, point, Math.toDegrees(heading));
    }
}
