package com.platapus.pathing.localization;

import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;

/**
 * An external device that tracks the robot's pose on its own.
 *
 * <p>A goBILDA Pinpoint or a SparkFun OTOS reads its own odometry pods, fuses
 * them with an onboard gyro, and hands back a finished pose. That is a different
 * thing from a raw sensor: there is nothing left for this library to filter, and
 * feeding an already-fused pose through {@link EKFPoseFuser} would be filtering
 * twice and trusting the result more for it.
 *
 * <p>So this interface is deliberately coarse -- a pose and a velocity, not
 * encoder counts. {@link OdometryComputerLocalizer} wraps it and the follower
 * cannot tell the difference from any other localizer.
 *
 * <p>No FTC SDK types appear here, so an implementation can be faked in a plain
 * JUnit test. The real adapter is roughly thirty lines and lives in your
 * TeamCode next to the vendor's driver -- see {@code examples/pinpoint/}.
 */
public interface OdometryComputer {

    /**
     * Reads the device. Called once per control loop, before anything asks for
     * a pose.
     *
     * <p>These devices are on I2C, so this is the expensive call and everything
     * else should be a cached read.
     */
    void refresh();

    /** The device's pose estimate, in field coordinates. */
    Pose2d getPose();

    /**
     * Robot-relative velocity: +x out the front, +y out the left.
     *
     * <p>If the device reports velocity in field coordinates, the adapter is
     * responsible for rotating it by {@code -heading} before returning it here.
     * Confirm which you have during bring-up: driving straight forward should
     * give a positive {@code vx} and a {@code vy} near zero, whichever way the
     * robot happens to be pointing.
     */
    ChassisSpeeds getVelocity();

    /**
     * Overrides the device's own pose estimate. Used to set the field start
     * pose before the auto begins.
     */
    void setPose(Pose2d pose);

    /** Whether the device is currently producing usable numbers. */
    Health getHealth();

    /**
     * A human-readable reason when {@link #getHealth()} is not {@link
     * Health#READY} -- which pod is missing, say. Surfaced on telemetry.
     */
    default String getHealthDetail() {
        return getHealth().name();
    }

    /**
     * What the device thinks of itself.
     *
     * <p>Deliberately coarser than any one vendor's status enum: this library
     * only needs to know whether to believe the pose, and a vendor-specific
     * fault code belongs in {@link #getHealthDetail()} where a human reads it.
     */
    enum Health {
        /** Producing good data. */
        READY,
        /** Powering up or calibrating its gyro. Poses are not yet meaningful. */
        CALIBRATING,
        /** A pod is unplugged, the gyro has run away, or a read failed. */
        FAULT,
        /** Not been read yet, or the device cannot say. */
        UNKNOWN
    }
}
