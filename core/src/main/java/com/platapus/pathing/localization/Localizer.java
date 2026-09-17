package com.platapus.pathing.localization;

import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;

/**
 * Whatever knows where the robot is.
 *
 * <p>In Phase 1 this is a single sensor source -- drive encoders, dead wheels, or
 * an external tracker. In Phase 2 the {@link PoseFuser} sits behind this same
 * interface and fuses several. The follower does not care which, and does not
 * change when the fusion layer lands.
 *
 * <p>All poses are in the field frame described on {@link
 * com.platapus.pathing.geometry.FieldCoordinates}.
 */
public interface Localizer {

    /**
     * Reads the sensors and advances the estimate. Called once per control loop,
     * before anything asks for the pose.
     */
    void update();

    /** The current best estimate of where the robot is. */
    Pose2d getPose();

    /**
     * Overrides the current estimate.
     *
     * <p>Prefer the localizer's {@code startPose} for the initial field pose --
     * it is applied before the first {@code update()}, so the IMU is calibrated
     * against it. This method exists for the rarer case of re-basing mid-match
     * against something you trust completely.
     *
     * <p>Not the way to fold in vision. {@link FusedLocalizer} takes an
     * observation as a weighted measurement update, precisely so that a bad
     * AprilTag read cannot teleport the robot mid-path.
     */
    void setPose(Pose2d pose);

    /** Robot-relative velocity, inches per second and radians per second. */
    ChassisSpeeds getVelocity();

    /**
     * The robot-relative movement measured over the most recent {@link #update()}.
     * The fuser consumes this as its prediction step.
     */
    default Twist2d getLastTwist() {
        return Twist2d.ZERO;
    }

    /**
     * How much to trust {@link #getPose()}, in [0, 1], where 1 is a tight
     * estimate.
     *
     * <p>A localizer with no notion of its own uncertainty reports 1, which is
     * the honest answer for dead reckoning: it has no idea how wrong it is. Only
     * {@link FusedLocalizer} returns anything else.
     *
     * <p>The follower reads this to decide how hard to correct. Reacting
     * aggressively toward a pose you do not believe is how a robot ends up
     * chasing its own estimation error across the field.
     */
    default double getConfidence() {
        return 1.0;
    }
}
