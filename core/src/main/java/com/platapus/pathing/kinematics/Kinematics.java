package com.platapus.pathing.kinematics;

import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Twist2d;

/**
 * The map between what the chassis is doing and what the individual wheels are
 * doing. Pure math -- no hardware, no FTC SDK, fully unit-testable.
 *
 * <p>Wheel arrays are always ordered front-left, front-right, back-left,
 * back-right for a four-wheel drive; a different geometry defines its own order
 * and documents it. Values are wheel-tangential inches per second.
 *
 * <p>This interface is what makes the follower drivetrain-agnostic: the follower
 * only ever produces a {@link ChassisSpeeds}, and the kinematics decides what
 * that means for the hardware. Tank and swerve slot in here without the follower
 * changing.
 */
public interface Kinematics {

    /** Number of wheel outputs this geometry has. */
    int getWheelCount();

    /**
     * Inverse kinematics: the wheel velocities that realise a chassis command.
     *
     * <p>For a geometry with fewer degrees of freedom than the command -- tank
     * being asked to strafe -- the unachievable component is dropped rather than
     * approximated. Check {@link #canStrafe()} before commanding lateral motion.
     */
    double[] toWheelVelocities(ChassisSpeeds speeds);

    /** Forward kinematics: what the chassis is doing, given the wheels. */
    ChassisSpeeds toChassisSpeeds(double[] wheelVelocities);

    /**
     * The per-wheel accelerations that realise a chassis acceleration.
     *
     * <p>The same map as {@link #toWheelVelocities}, and deliberately so: the
     * kinematics are linear, so the matrix that turns a chassis velocity into
     * wheel velocities turns its derivative into wheel accelerations
     * unchanged. Naming it separately is not redundancy -- it is so a reader
     * meeting {@code toWheelVelocities(someAcceleration)} at a call site does
     * not have to stop and work out whether it is a bug.
     *
     * <p>An implementation with non-linear kinematics -- swerve, where module
     * angles matter -- would need to override this rather than inherit it.
     */
    default double[] toWheelAccelerations(ChassisSpeeds chassisAccelerations) {
        return toWheelVelocities(chassisAccelerations);
    }

    /**
     * Forward kinematics on positions rather than velocities: converts per-wheel
     * distance travelled over one loop into a robot-relative movement. This is
     * how drive-encoder odometry gets its twist.
     */
    default Twist2d toTwist(double[] wheelDeltas) {
        ChassisSpeeds s = toChassisSpeeds(wheelDeltas);
        return new Twist2d(s.vx, s.vy, s.omega);
    }

    /** Whether this geometry can translate sideways without rotating. */
    boolean canStrafe();

    /**
     * The fastest the chassis can go forwards, inches per second, given the
     * wheels' own top speed. Used by the profile as its velocity ceiling.
     */
    double getMaxLinearVelocity();

    /**
     * Scales wheel velocities into motor powers in [-1, 1].
     *
     * <p>When any wheel exceeds its top speed the whole set is scaled down
     * together rather than clipped individually. Clipping one wheel changes the
     * ratio between them, which changes the direction the robot actually
     * travels -- the classic reason a robot drifts off a path only when driving
     * fast. Scaling preserves the commanded direction and just makes it slower.
     */
    default double[] normalize(double[] wheelVelocities) {
        double max = 0.0;
        for (double v : wheelVelocities) {
            max = Math.max(max, Math.abs(v));
        }

        double top = getMaxWheelVelocity();
        double divisor = Math.max(top, max);
        double[] out = new double[wheelVelocities.length];
        if (divisor < 1e-9) {
            return out;
        }
        for (int i = 0; i < wheelVelocities.length; i++) {
            out[i] = wheelVelocities[i] / divisor;
        }
        return out;
    }

    /** Top tangential speed of a single wheel, inches per second. */
    double getMaxWheelVelocity();
}
