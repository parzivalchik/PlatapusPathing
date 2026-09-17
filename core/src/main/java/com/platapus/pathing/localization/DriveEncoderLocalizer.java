package com.platapus.pathing.localization;

import com.platapus.pathing.control.Clock;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.geometry.Angles;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.kinematics.Kinematics;

/**
 * Odometry from the drive motors' own encoders, optionally with the IMU
 * supplying heading.
 *
 * <p>This is the baseline localizer: it needs no dead wheels and no extra
 * hardware. It is also the least accurate one, because mecanum wheels slip
 * laterally and the encoders cannot see it -- expect drift to accumulate over a
 * long auto. Supplying a {@link HeadingSource} fixes the worst of it, since
 * heading error is what turns small translation errors into large ones.
 *
 * <p>{@link FusedLocalizer} replaces the heading override here with a proper
 * measurement update, so the IMU informs the estimate in proportion to its
 * trustworthiness rather than simply overwriting it.
 *
 * <p>Poses are in the field frame described by {@link
 * com.platapus.pathing.geometry.FieldCoordinates}, so this must be told where
 * the robot starts -- it cannot work that out from the encoders. Use the
 * builder's {@code startPose}.
 */
public final class DriveEncoderLocalizer implements Localizer {

    private final Drivetrain drivetrain;
    private final Kinematics kinematics;
    private final Clock clock;
    private final HeadingSource headingSource;

    private Pose2d pose;
    private double[] lastWheelPositions;
    private double lastTime = Double.NaN;
    private double headingOffset = Double.NaN;
    private ChassisSpeeds velocity = ChassisSpeeds.ZERO;
    private Twist2d lastTwist = Twist2d.ZERO;

    private DriveEncoderLocalizer(Builder b) {
        this.drivetrain = b.drivetrain;
        this.kinematics = b.drivetrain.getKinematics();
        this.clock = b.clock;
        this.headingSource = b.headingSource;
        this.pose = b.startPose;
    }

    public static Builder builder(Drivetrain drivetrain, Clock clock) {
        return new Builder(drivetrain, clock);
    }

    public static final class Builder {
        private final Drivetrain drivetrain;
        private final Clock clock;
        private HeadingSource headingSource;
        private Pose2d startPose = Pose2d.ZERO;

        private Builder(Drivetrain drivetrain, Clock clock) {
            if (drivetrain == null || clock == null) {
                throw new IllegalArgumentException("drivetrain and clock must be non-null");
            }
            this.drivetrain = drivetrain;
            this.clock = clock;
        }

        /**
         * Where the robot actually is when the auto begins, in field
         * coordinates. Defaults to the field centre facing +X, which is almost
         * certainly not where your robot is.
         *
         * <p>Get this wrong and every path is offset by the same error -- the
         * robot drives the right shape in the wrong place.
         */
        public Builder startPose(Pose2d startPose) {
            if (startPose == null) {
                throw new IllegalArgumentException("startPose must be non-null");
            }
            this.startPose = startPose;
            return this;
        }

        /** The IMU. Without it, heading is integrated from the wheels alone. */
        public Builder headingSource(HeadingSource source) {
            this.headingSource = source;
            return this;
        }

        public DriveEncoderLocalizer build() {
            return new DriveEncoderLocalizer(this);
        }
    }

    @Override
    public void update() {
        double now = clock.seconds();
        double[] positions = drivetrain.getWheelPositions();

        if (lastWheelPositions == null) {
            lastWheelPositions = positions.clone();
            lastTime = now;
            if (headingSource != null) {
                // The gyro's reading at the first update is calibrated against
                // the configured start heading, which is what puts the estimate
                // in field coordinates rather than relative to power-on.
                headingOffset = headingSource.getHeadingRadians() - pose.heading;
            }
            return;
        }

        double[] deltas = new double[positions.length];
        for (int i = 0; i < positions.length; i++) {
            deltas[i] = positions[i] - lastWheelPositions[i];
        }
        lastWheelPositions = positions.clone();

        Twist2d twist = kinematics.toTwist(deltas);

        if (headingSource != null) {
            // Trust the gyro over the wheels for rotation: wheel-derived heading
            // drifts fastest of anything here.
            double measured = Angles.normalize(headingSource.getHeadingRadians() - headingOffset);
            double gyroDelta = Angles.normalize(measured - pose.heading);
            twist = new Twist2d(twist.dx, twist.dy, gyroDelta);
        }

        lastTwist = twist;
        pose = twist.applyTo(pose);

        double dt = now - lastTime;
        lastTime = now;
        if (dt > 1e-9) {
            velocity = new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dTheta / dt);
        }
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
        if (headingSource != null) {
            headingOffset = headingSource.getHeadingRadians() - pose.heading;
        }
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    @Override
    public Twist2d getLastTwist() {
        return lastTwist;
    }
}
