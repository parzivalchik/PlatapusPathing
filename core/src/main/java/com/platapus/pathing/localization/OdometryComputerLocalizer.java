package com.platapus.pathing.localization;

import com.platapus.pathing.geometry.Angles;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;

/**
 * Localization from an {@link OdometryComputer} -- a goBILDA Pinpoint, a
 * SparkFun OTOS, or anything else that tracks its own pose.
 *
 * <p>The most accurate option this library offers, and the simplest: the device
 * has already fused its pods with its own gyro, so there is nothing here but
 * reading it, watching its health, and reporting a pose the follower can use.
 *
 * <p><b>No filtering is applied on top.</b> {@link FusedLocalizer} exists to
 * combine a drivetrain's own noisy encoders with a separate gyro; running an
 * already-fused pose through it again would smooth an estimate that is better
 * than anything the extra filter knows about, add lag, and -- worse -- shrink
 * the reported covariance as though two independent measurements had agreed,
 * when there is only one.
 *
 * <h2>Health</h2>
 * These devices fail in ways drive encoders cannot: a pod cable pulls out, the
 * gyro runs away, an I2C read is dropped. When that happens the pose is stale or
 * wrong but still <i>numerically plausible</i>, which is the dangerous case.
 *
 * <p>So {@link #getConfidence()} drops to zero on a fault and the follower
 * scales its correction authority back accordingly, rather than driving hard
 * toward a position the robot is not at. The last good pose is held rather than
 * replaced by garbage, and {@link #getHealthDetail()} says what went wrong.
 * Whether that should abort the auto is the OpMode's decision, not this class's.
 */
public final class OdometryComputerLocalizer implements Localizer {

    private final OdometryComputer device;

    private Pose2d pose;
    private Pose2d lastGoodPose;
    private ChassisSpeeds velocity = ChassisSpeeds.ZERO;
    private Twist2d lastTwist = Twist2d.ZERO;
    private OdometryComputer.Health health = OdometryComputer.Health.UNKNOWN;
    private boolean sawFault;

    private OdometryComputerLocalizer(Builder b) {
        this.device = b.device;
        this.pose = b.startPose;
        this.lastGoodPose = b.startPose;
        // The device carries its own idea of where it is, usually left over from
        // the previous OpMode. Tell it where the robot actually starts before
        // anything reads it.
        this.device.setPose(b.startPose);
    }

    public static Builder builder(OdometryComputer device) {
        return new Builder(device);
    }

    public static final class Builder {
        private final OdometryComputer device;
        private Pose2d startPose = Pose2d.ZERO;

        private Builder(OdometryComputer device) {
            if (device == null) {
                throw new IllegalArgumentException("device must be non-null");
            }
            this.device = device;
        }

        /**
         * Where the robot is placed at the start of the auto, in field
         * coordinates. Written straight through to the device.
         */
        public Builder startPose(Pose2d startPose) {
            if (startPose == null) {
                throw new IllegalArgumentException("startPose must be non-null");
            }
            this.startPose = startPose;
            return this;
        }

        public OdometryComputerLocalizer build() {
            return new OdometryComputerLocalizer(this);
        }
    }

    @Override
    public void update() {
        device.refresh();
        health = device.getHealth() == null
                ? OdometryComputer.Health.UNKNOWN : device.getHealth();

        if (health != OdometryComputer.Health.READY) {
            // Hold the last pose we believed rather than accepting a reading the
            // device itself is disowning. A stale pose is wrong in a way the
            // follower can fight; a garbage one is wrong in a way it cannot.
            sawFault |= health == OdometryComputer.Health.FAULT;
            velocity = ChassisSpeeds.ZERO;
            lastTwist = Twist2d.ZERO;
            pose = lastGoodPose;
            return;
        }

        Pose2d reported = device.getPose();
        if (reported == null || !isFinite(reported)) {
            sawFault = true;
            health = OdometryComputer.Health.FAULT;
            pose = lastGoodPose;
            velocity = ChassisSpeeds.ZERO;
            lastTwist = Twist2d.ZERO;
            return;
        }

        // The movement since the previous good reading, in the robot's own frame
        // at the start of the interval -- the same thing a wheel localizer
        // reports, so anything downstream that wants a twist still gets one.
        lastTwist = twistBetween(pose, reported);
        pose = reported;
        lastGoodPose = reported;

        ChassisSpeeds reportedVelocity = device.getVelocity();
        velocity = reportedVelocity == null ? ChassisSpeeds.ZERO : reportedVelocity;
    }

    /** The robot-frame movement from {@code from} to {@code to}. */
    private static Twist2d twistBetween(Pose2d from, Pose2d to) {
        Pose2d relative = to.relativeTo(from);
        return new Twist2d(relative.position.x, relative.position.y,
                Angles.normalize(to.heading - from.heading));
    }

    private static boolean isFinite(Pose2d p) {
        return Double.isFinite(p.position.x)
                && Double.isFinite(p.position.y)
                && Double.isFinite(p.heading);
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            throw new IllegalArgumentException("pose must be non-null");
        }
        this.pose = pose;
        this.lastGoodPose = pose;
        device.setPose(pose);
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    @Override
    public Twist2d getLastTwist() {
        return lastTwist;
    }

    /**
     * 1 while the device is healthy, 0 otherwise.
     *
     * <p>Binary rather than graded on purpose. A pose tracker of this kind does
     * not degrade gently -- it is either tracking or it has lost a pod -- so a
     * middling number would be inventing a precision the device does not report.
     */
    @Override
    public double getConfidence() {
        return health == OdometryComputer.Health.READY ? 1.0 : 0.0;
    }

    /** The device's current health. */
    public OdometryComputer.Health getHealth() {
        return health;
    }

    /** A human-readable health line, for telemetry. */
    public String getHealthDetail() {
        return device.getHealthDetail();
    }

    /**
     * Whether the device has faulted at any point since this localizer was
     * built.
     *
     * <p>A dropped pod that reconnects leaves the pose permanently offset by
     * however far the robot moved while it was out, and the device will report
     * READY again as though nothing happened. This latches so an OpMode can tell
     * the difference between "fine" and "fine now".
     */
    public boolean hasFaulted() {
        return sawFault;
    }

    /** Clears the latched fault flag, after you have dealt with it. */
    public void clearFaultLatch() {
        sawFault = false;
    }
}
