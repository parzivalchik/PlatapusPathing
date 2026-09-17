package com.platapus.pathing.localization;

import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;

/**
 * Combines several sources of pose information into one estimate.
 *
 * <p><b>Phase 2 will implement this as an EKF.</b> The interface is declared now
 * so the follower and the localizer contract are already the right shape and do
 * not have to change when the filter lands.
 *
 * <p>The design point is that vision arrives as a <i>measurement update</i>,
 * weighted by its own covariance, rather than as a {@code resetPose()} call. A
 * single marginal AprilTag read at the edge of the frame then nudges the estimate
 * in proportion to how much it should be trusted, instead of snapping the robot's
 * believed position mid-path.
 */
public interface PoseFuser {

    /**
     * Prediction step: advances the estimate by a measured odometry increment and
     * grows the uncertainty accordingly.
     */
    void predict(Twist2d odometryDelta, double dtSeconds);

    /**
     * Measurement update from a heading sensor, typically the IMU.
     *
     * @param headingRad measured heading, radians CCW from field +X
     * @param variance   the measurement's variance, radians squared. Larger means
     *                   trusted less.
     */
    void correctHeading(double headingRad, double variance);

    /**
     * Measurement update from an absolute pose observation -- AprilTags, once
     * Phase 3 wires them up.
     *
     * @param observed  the observed pose, in field coordinates
     * @param variance  {x, y, heading} variances, inches squared and radians squared
     */
    void correctPose(Pose2d observed, double[] variance);

    /** The fused estimate. */
    Pose2d getPose();

    /**
     * A scalar summary of how much to trust {@link #getPose()}, in [0, 1], where
     * 1 is a tight estimate. Derived from the filter's covariance; intended for
     * telemetry and for the Phase 2 reactive blend, which should not fight hard
     * to correct toward a pose it is not confident about.
     */
    double getConfidence();

    /** Discards all state and restarts the filter at {@code pose}. */
    void reset(Pose2d pose);
}
