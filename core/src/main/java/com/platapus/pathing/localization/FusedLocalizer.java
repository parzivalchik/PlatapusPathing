package com.platapus.pathing.localization;

import com.platapus.pathing.control.Clock;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.kinematics.Kinematics;

/**
 * The Phase 2 localizer: wheel odometry predicts, the IMU corrects, and vision
 * corrects too once something implements {@link VisionPoseSource}.
 *
 * <p>Drop-in replacement for {@link DriveEncoderLocalizer} -- same interface, and
 * the follower cannot tell the difference. The distinction is in how the IMU is
 * used. The encoder localizer simply overwrites wheel-derived rotation with the
 * gyro's. This one treats the gyro as a measurement with a variance, so a
 * momentarily confused IMU nudges the estimate instead of dictating it, and the
 * filter also learns something about <i>position</i> from a heading measurement
 * through the correlations the covariance carries.
 */
public final class FusedLocalizer implements Localizer {

    private final Drivetrain drivetrain;
    private final Kinematics kinematics;
    private final Clock clock;
    private final HeadingSource headingSource;
    private final VisionPoseSource visionSource;
    private final EKFPoseFuser fuser;
    private final double headingVariance;

    private double[] lastWheelPositions;
    private double lastTime = Double.NaN;
    private double headingOffset = Double.NaN;
    private ChassisSpeeds velocity = ChassisSpeeds.ZERO;
    private Twist2d lastTwist = Twist2d.ZERO;
    private double lastVisionTimestamp = Double.NaN;

    private FusedLocalizer(Builder b) {
        this.drivetrain = b.drivetrain;
        this.kinematics = b.drivetrain.getKinematics();
        this.clock = b.clock;
        this.headingSource = b.headingSource;
        this.visionSource = b.visionSource;
        this.headingVariance = b.headingVariance;
        this.fuser = b.fuser != null ? b.fuser : EKFPoseFuser.builder().build();
        // The filter has to begin at the robot's real field pose; absolute
        // coordinates cannot be inferred from the encoders.
        this.fuser.reset(b.startPose);
    }

    public static Builder builder(Drivetrain drivetrain, Clock clock) {
        return new Builder(drivetrain, clock);
    }

    public static final class Builder {
        private final Drivetrain drivetrain;
        private final Clock clock;
        private HeadingSource headingSource;
        private VisionPoseSource visionSource;
        private EKFPoseFuser fuser;
        private Pose2d startPose = Pose2d.ZERO;
        private double headingVariance = 1e-4; // about 0.6 degrees of sigma

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

        /** The IMU. Optional, but without it this is just odometry with extra steps. */
        public Builder headingSource(HeadingSource source) {
            this.headingSource = source;
            return this;
        }

        /**
         * How much to trust the heading source, radians squared. Smaller means
         * more trusted. The default assumes a Control Hub IMU that is good to
         * well under a degree over an auto.
         */
        public Builder headingVariance(double variance) {
            this.headingVariance = variance;
            return this;
        }

        /**
         * An absolute pose source. Nothing implements {@link VisionPoseSource}
         * yet -- see its javadoc.
         */
        public Builder visionSource(VisionPoseSource source) {
            this.visionSource = source;
            return this;
        }

        /** Supply a pre-configured filter instead of the default one. */
        public Builder fuser(EKFPoseFuser fuser) {
            this.fuser = fuser;
            return this;
        }

        public FusedLocalizer build() {
            return new FusedLocalizer(this);
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
                // Calibrate the gyro against the configured start heading, so
                // its readings land in field coordinates.
                headingOffset = headingSource.getHeadingRadians() - fuser.getPose().heading;
            }
            return;
        }

        double[] deltas = new double[positions.length];
        for (int i = 0; i < positions.length; i++) {
            deltas[i] = positions[i] - lastWheelPositions[i];
        }
        lastWheelPositions = positions.clone();

        double dt = now - lastTime;
        lastTime = now;

        // 1. Predict from odometry.
        Twist2d twist = kinematics.toTwist(deltas);
        lastTwist = twist;
        fuser.predict(twist, dt);

        // 2. Correct with heading.
        if (headingSource != null) {
            double measured = headingSource.getHeadingRadians() - headingOffset;
            fuser.correctHeading(measured, headingVariance);
        }

        // 3. Correct with vision, when there is anything to correct with.
        if (visionSource != null) {
            VisionPoseSource.Observation observation = visionSource.getObservation();
            if (observation != null && isFresh(observation)) {
                lastVisionTimestamp = observation.timestamp;
                fuser.correctPose(observation.pose, observation.variance);
            }
        }

        if (dt > 1e-9) {
            velocity = new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dTheta / dt);
        }
    }

    /**
     * Rejects an observation this localizer has already folded in.
     *
     * <p>A camera pipeline usually runs slower than the control loop, so the same
     * detection is handed over on several consecutive calls. Applying it more
     * than once would let one measurement shrink the covariance repeatedly and
     * convince the filter it is far more certain than it has any right to be.
     */
    private boolean isFresh(VisionPoseSource.Observation observation) {
        return Double.isNaN(lastVisionTimestamp)
                || observation.timestamp > lastVisionTimestamp;
    }

    @Override
    public Pose2d getPose() {
        return fuser.getPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        if (pose == null) {
            throw new IllegalArgumentException("pose must be non-null");
        }
        fuser.reset(pose);
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

    @Override
    public double getConfidence() {
        return fuser.getConfidence();
    }

    /** The filter itself, for telemetry or for feeding vision in by hand. */
    public EKFPoseFuser getFuser() {
        return fuser;
    }
}
