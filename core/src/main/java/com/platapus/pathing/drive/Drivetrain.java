package com.platapus.pathing.drive;

import com.platapus.pathing.kinematics.Kinematics;

/**
 * The seam between the follower and the actual motors.
 *
 * <p>Deliberately tiny. Everything a drivetrain needs to know how to do is here,
 * and nothing else -- no path logic, no control loops, no pose. The FTC-specific
 * implementation in the {@code :ftc} module is roughly forty lines of {@code
 * DcMotorEx} calls; this interface is what keeps it that small, and what lets the
 * whole follower be tested against a fake in plain JUnit.
 *
 * <p>Wheel arrays follow the order defined by this drivetrain's {@link
 * Kinematics} -- front-left, front-right, back-left, back-right for mecanum.
 */
public interface Drivetrain {

    /** The geometry of this drivetrain. Never null, never changes. */
    Kinematics getKinematics();

    /**
     * Commands motor powers in [-1, 1], one per wheel.
     *
     * <p>The follower has already normalised these; an implementation should
     * write them through rather than re-clipping or re-scaling.
     */
    void setWheelPowers(double[] powers);

    /**
     * Cumulative distance each wheel has travelled, inches, since whenever the
     * implementation zeroed. Only differences between consecutive reads are ever
     * used, so the absolute value and the starting point do not matter.
     *
     * <p>A drivetrain with no encoders -- or one on a robot with dead wheels,
     * where the drive encoders are not the odometry source -- may throw {@link
     * UnsupportedOperationException} here as long as a separate localizer is
     * supplying the pose.
     */
    double[] getWheelPositions();

    /**
     * Measured wheel velocities, inches per second.
     *
     * <p>Defaults to unsupported: velocity is only needed by the auto-tuner in
     * Phase 3, and most drivetrains can get by without it.
     */
    default double[] getWheelVelocities() {
        throw new UnsupportedOperationException(
                getClass().getSimpleName() + " does not report wheel velocities");
    }

    /** Cuts power to every wheel. Must be safe to call repeatedly. */
    default void stop() {
        setWheelPowers(new double[getKinematics().getWheelCount()]);
    }
}
