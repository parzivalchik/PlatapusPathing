package com.platapus.pathing.sim;

import com.platapus.pathing.control.Clock;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.kinematics.Kinematics;
import com.platapus.pathing.localization.Localizer;

/**
 * A kinematic stand-in for a real robot, for testing the follower without
 * hardware.
 *
 * <p>Motor powers are taken to produce wheel velocities instantly and
 * proportionally -- there is no motor dynamics model and no battery sag, both of
 * which are explicitly out of scope. That makes this a test of the follower's
 * <i>geometry and control structure</i>, not of its tuning: it proves the
 * follower commands the right direction and converges, not that any particular
 * gain is right for a real robot.
 *
 * <p>The simulation exposes itself as both the {@link Drivetrain} the follower
 * commands and the {@link Localizer} it reads back, closing the loop.
 */
public final class SimulatedRobot implements Drivetrain, Localizer {

    private final Kinematics kinematics;
    private final ManualClock clock;

    private Pose2d pose;
    private double[] wheelPowers;
    private double[] wheelPositions;
    private ChassisSpeeds velocity = ChassisSpeeds.ZERO;
    private Twist2d lastTwist = Twist2d.ZERO;

    /** Multiplies commanded wheel speed, to simulate a robot that under-delivers. */
    private double powerScale = 1.0;
    /** Constant world-frame velocity added every step, to simulate a push. */
    private ChassisSpeeds disturbance = ChassisSpeeds.ZERO;

    public SimulatedRobot(Kinematics kinematics, ManualClock clock, Pose2d startPose) {
        this.kinematics = kinematics;
        this.clock = clock;
        this.pose = startPose;
        this.wheelPowers = new double[kinematics.getWheelCount()];
        this.wheelPositions = new double[kinematics.getWheelCount()];
    }

    /**
     * Advances the physical simulation by {@code dt} seconds and moves the clock
     * with it. Call once per control loop, after {@code follower.update()}.
     */
    public void step(double dt) {
        double[] wheelVelocities = new double[wheelPowers.length];
        for (int i = 0; i < wheelPowers.length; i++) {
            wheelVelocities[i] = wheelPowers[i] * kinematics.getMaxWheelVelocity() * powerScale;
            wheelPositions[i] += wheelVelocities[i] * dt;
        }

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(wheelVelocities).plus(disturbance);
        velocity = speeds;
        Twist2d twist = new Twist2d(speeds.vx * dt, speeds.vy * dt, speeds.omega * dt);
        pose = twist.applyTo(pose);
        clock.advance(dt);
    }

    /** Teleports the robot, to simulate being shoved off the path. */
    public void displace(double dx, double dy, double dHeading) {
        pose = new Pose2d(pose.getX() + dx, pose.getY() + dy, pose.getHeading() + dHeading);
    }

    /** Sets how much of the commanded wheel speed the robot actually delivers. */
    public void setPowerScale(double scale) {
        this.powerScale = scale;
    }

    /** Adds a constant robot-relative velocity disturbance every step. */
    public void setDisturbance(ChassisSpeeds disturbance) {
        this.disturbance = disturbance;
    }

    /** The true pose, as opposed to whatever a localizer might estimate. */
    public Pose2d getTruePose() {
        return pose;
    }

    public double[] getLastPowers() {
        return wheelPowers.clone();
    }

    // --- Drivetrain ---

    @Override
    public Kinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void setWheelPowers(double[] powers) {
        this.wheelPowers = powers.clone();
    }

    @Override
    public double[] getWheelPositions() {
        return wheelPositions.clone();
    }

    @Override
    public double[] getWheelVelocities() {
        double[] out = new double[wheelPowers.length];
        for (int i = 0; i < wheelPowers.length; i++) {
            out[i] = wheelPowers[i] * kinematics.getMaxWheelVelocity() * powerScale;
        }
        return out;
    }

    // --- Localizer: perfect knowledge, so follower behaviour is not confounded
    // with odometry error. Odometry itself is tested separately. ---

    @Override
    public void update() {
        // The pose is already exact; nothing to estimate.
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    @Override
    public Twist2d getLastTwist() {
        return lastTwist;
    }

    /** A clock the test advances by hand. */
    public static final class ManualClock implements Clock {
        private double now;

        @Override
        public double seconds() {
            return now;
        }

        public void advance(double dt) {
            now += dt;
        }
    }
}
