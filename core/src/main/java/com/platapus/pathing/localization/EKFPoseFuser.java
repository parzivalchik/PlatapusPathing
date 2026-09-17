package com.platapus.pathing.localization;

import com.platapus.pathing.geometry.Angles;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.math.Matrix3;

/**
 * An extended Kalman filter over {@code [x, y, heading]}.
 *
 * <p>Odometry enters as the prediction step's control input, and everything else
 * -- the IMU, and later AprilTags -- enters as a measurement update weighted by
 * its own variance. That distinction is the point of the whole class. The usual
 * FTC approach of calling {@code resetPose()} when a tag comes into view throws
 * away a good odometry estimate in favour of a single frame's observation, and
 * teleports the robot mid-path when that observation is marginal. Here a
 * measurement moves the estimate in proportion to how much it deserves to.
 *
 * <p>Not thread-safe; call it from the control loop only.
 *
 * <h2>Why the simple covariance update, not Joseph form</h2>
 * The updates below use {@code P = (I - KH)P} and re-symmetrise afterwards.
 * The Joseph form, {@code P = (I-KH)P(I-KH)' + KRK'}, is more expensive but
 * guarantees the covariance stays positive-definite however long it runs, where
 * this form can in principle drift negative through accumulated rounding.
 *
 * <p>That guarantee is worth having in a filter that runs for hours. An FTC
 * autonomous runs for thirty seconds, and a 5000-step test asserts the variances
 * stay positive and finite across far more updates than a match contains. The
 * simple form was chosen for that reason, not overlooked.
 *
 * <p>If you ever do see this misbehave -- most likely after an unusually long
 * tuning session rather than in a match -- switching the two update methods to
 * Joseph form is the first thing to try.
 *
 * <h2>Frames</h2>
 * Everything is in field coordinates -- see {@link
 * com.platapus.pathing.geometry.FieldCoordinates}.
 */
public final class EKFPoseFuser implements PoseFuser {

    private final double translationalNoisePerInch;
    private final double lateralNoisePerInch;
    private final double headingNoisePerRadian;
    private final double baseNoisePerSecond;
    private final double confidenceScale;

    private Pose2d pose;
    private Matrix3 covariance;

    private EKFPoseFuser(Builder b) {
        this.translationalNoisePerInch = b.translationalNoisePerInch;
        this.lateralNoisePerInch = b.lateralNoisePerInch;
        this.headingNoisePerRadian = b.headingNoisePerRadian;
        this.baseNoisePerSecond = b.baseNoisePerSecond;
        this.confidenceScale = b.confidenceScale;
        this.pose = b.initialPose;
        this.covariance = Matrix3.diagonal(
                b.initialVariance[0], b.initialVariance[1], b.initialVariance[2]);
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private double translationalNoisePerInch = 0.002;
        private double lateralNoisePerInch = 0.010;
        private double headingNoisePerRadian = 0.020;
        private double baseNoisePerSecond = 1e-6;
        private double confidenceScale = 2.0;
        private Pose2d initialPose = Pose2d.ZERO;
        private double[] initialVariance = {1e-6, 1e-6, 1e-6};

        /**
         * Variance added per inch travelled along the robot's forward axis,
         * inches squared. Small: wheels measure forward travel well.
         */
        public Builder translationalNoisePerInch(double v) {
            this.translationalNoisePerInch = v;
            return this;
        }

        /**
         * Variance added per inch of lateral travel, inches squared.
         *
         * <p>Deliberately larger than the forward figure by default. Mecanum
         * wheels scrub sideways and the encoders cannot see it, so lateral
         * odometry is the least trustworthy thing the filter is fed. Telling the
         * filter that is what lets a heading or vision measurement pull the
         * estimate sideways without also corrupting the along-track estimate.
         */
        public Builder lateralNoisePerInch(double v) {
            this.lateralNoisePerInch = v;
            return this;
        }

        /** Variance added per radian of rotation, radians squared. */
        public Builder headingNoisePerRadian(double v) {
            this.headingNoisePerRadian = v;
            return this;
        }

        /**
         * Variance added per second regardless of motion. Keeps the covariance
         * from collapsing to zero while the robot sits still, which would make
         * the filter ignore every subsequent measurement.
         */
        public Builder baseNoisePerSecond(double v) {
            this.baseNoisePerSecond = v;
            return this;
        }

        /**
         * The position standard deviation, inches, at which {@link
         * #getConfidence()} reads about 0.5. Purely a reporting knob -- it has no
         * effect on the estimate itself.
         */
        public Builder confidenceScale(double inches) {
            this.confidenceScale = inches;
            return this;
        }

        public Builder initialPose(Pose2d pose) {
            this.initialPose = pose;
            return this;
        }

        /** Starting uncertainty as {x, y, heading} variances. */
        public Builder initialVariance(double x, double y, double heading) {
            this.initialVariance = new double[]{x, y, heading};
            return this;
        }

        public EKFPoseFuser build() {
            return new EKFPoseFuser(this);
        }
    }

    @Override
    public void predict(Twist2d odometryDelta, double dtSeconds) {
        if (odometryDelta == null) {
            throw new IllegalArgumentException("odometryDelta must be non-null");
        }
        double dt = Math.max(0.0, dtSeconds);
        double heading = pose.heading;

        // Mean: the same exact-arc integration the plain odometry localizer uses,
        // so the filter's prediction is never worse than not filtering at all.
        Pose2d predicted = odometryDelta.applyTo(pose);
        Vector2d worldDelta = predicted.position.minus(pose.position);

        // Jacobian of the motion model with respect to the state. Position
        // depends on heading only through the rotation of the local displacement,
        // whose derivative is that displacement turned 90 degrees.
        Matrix3 f = Matrix3.of(
                1, 0, -worldDelta.y,
                0, 1, worldDelta.x,
                0, 0, 1);

        covariance = f.times(covariance).times(f.transpose())
                .plus(processNoise(odometryDelta, heading, dt))
                .symmetrized();
        pose = predicted;
    }

    /**
     * Process noise for one step, in world coordinates.
     *
     * <p>Built in the robot's own frame -- where "forward" and "sideways" are
     * meaningful and have genuinely different error characteristics -- then
     * rotated into the world frame. Building it directly in world coordinates
     * would smear the good along-track estimate into the bad lateral one and
     * lose exactly the information that makes the filter worth having.
     */
    private Matrix3 processNoise(Twist2d twist, double heading, double dt) {
        // ErrordaForever: even a robot sitting perfectly still accrues this
        // much doubt per second. A filter that ever became certain would stop
        // listening, and a filter that stops listening is a lookup table.
        double base = baseNoisePerSecond * dt;
        double along = translationalNoisePerInch * Math.abs(twist.dx) + base;
        double lateral = lateralNoisePerInch * Math.abs(twist.dy) + base;
        double angular = headingNoisePerRadian * Math.abs(twist.dTheta) + base;

        double c = Math.cos(heading);
        double s = Math.sin(heading);
        // R * diag(along, lateral) * R^T, written out.
        double qxx = along * c * c + lateral * s * s;
        double qxy = (along - lateral) * c * s;
        double qyy = along * s * s + lateral * c * c;

        return Matrix3.of(
                qxx, qxy, 0,
                qxy, qyy, 0,
                0, 0, angular);
    }

    @Override
    public void correctHeading(double headingRad, double variance) {
        if (variance <= 0) {
            throw new IllegalArgumentException("variance must be positive, got " + variance);
        }

        // H = [0 0 1], so H*P is just P's bottom row and H*P*H^T is P[2][2].
        double innovation = Angles.normalize(headingRad - pose.heading);
        double s = covariance.get(2, 2) + variance;

        double k0 = covariance.get(0, 2) / s;
        double k1 = covariance.get(1, 2) / s;
        double k2 = covariance.get(2, 2) / s;

        pose = new Pose2d(
                pose.position.x + k0 * innovation,
                pose.position.y + k1 * innovation,
                Angles.normalize(pose.heading + k2 * innovation));

        // P = (I - K H) P. K H has a single non-zero column, so the product
        // reduces to subtracting the outer product of K with P's bottom row.
        Matrix3 update = Matrix3.of(
                k0 * covariance.get(2, 0), k0 * covariance.get(2, 1), k0 * covariance.get(2, 2),
                k1 * covariance.get(2, 0), k1 * covariance.get(2, 1), k1 * covariance.get(2, 2),
                k2 * covariance.get(2, 0), k2 * covariance.get(2, 1), k2 * covariance.get(2, 2));
        covariance = covariance.minus(update).symmetrized();
    }

    @Override
    public void correctPose(Pose2d observed, double[] variance) {
        if (observed == null) {
            throw new IllegalArgumentException("observed must be non-null");
        }
        if (variance == null || variance.length != 3) {
            throw new IllegalArgumentException("variance must be {x, y, heading}");
        }
        for (double v : variance) {
            if (v <= 0) {
                throw new IllegalArgumentException("variances must be positive");
            }
        }

        // H = I for a full pose observation, so the update is the textbook form
        // with the identity dropped out.
        double[] innovation = {
                observed.position.x - pose.position.x,
                observed.position.y - pose.position.y,
                Angles.normalize(observed.heading - pose.heading)
        };

        Matrix3 s = covariance.plus(
                Matrix3.diagonal(variance[0], variance[1], variance[2]));
        Matrix3 k = covariance.times(s.inverse());

        double[] correction = k.times(innovation);
        pose = new Pose2d(
                pose.position.x + correction[0],
                pose.position.y + correction[1],
                Angles.normalize(pose.heading + correction[2]));

        covariance = Matrix3.identity().minus(k).times(covariance).symmetrized();
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public double getConfidence() {
        // A bounded, monotonic summary of positional uncertainty. Reads 1 when
        // the estimate is tight and decays toward 0 as it loosens. Heuristic --
        // it drives telemetry and the follower's correction authority, never the
        // estimate itself.
        double positionVariance = covariance.get(0, 0) + covariance.get(1, 1);
        double scale = confidenceScale * confidenceScale;
        return scale / (scale + positionVariance);
    }

    @Override
    public void reset(Pose2d pose) {
        if (pose == null) {
            throw new IllegalArgumentException("pose must be non-null");
        }
        this.pose = pose;
        this.covariance = Matrix3.diagonal(1e-6, 1e-6, 1e-6);
    }

    /** The {x, y, heading} variances on the diagonal of the covariance. */
    public double[] getVariance() {
        return new double[]{
                covariance.get(0, 0), covariance.get(1, 1), covariance.get(2, 2)};
    }

    /** Standard deviation of the position estimate, inches. */
    public double getPositionStdDev() {
        return Math.sqrt(covariance.get(0, 0) + covariance.get(1, 1));
    }

    /** Standard deviation of the heading estimate, radians. */
    public double getHeadingStdDev() {
        return Math.sqrt(covariance.get(2, 2));
    }
}
