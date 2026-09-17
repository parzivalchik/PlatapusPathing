package com.platapus.pathing.control;

import com.platapus.pathing.units.DistanceUnit;

/**
 * Every number the follower needs to tune, in one immutable object.
 *
 * <p>Distances are given in whatever {@link DistanceUnit} the builder is created
 * with and stored canonically; angles are always radians.
 *
 * <p>Feedforward convention: motor power for a wheel is
 * {@code kS * sign(v) + kV * v + kA * a}, with {@code v} in inches per second and
 * {@code a} in inches per second squared. So {@code kV} is roughly
 * {@code 1 / maxWheelVelocity} for a robot that reaches top speed at full power,
 * and that is a good first guess before any tuning. Phase 3's auto-tuner fits
 * these three online.
 */
public final class FollowerConstants {

    private final double maxVelocity;
    private final double maxAcceleration;
    private final double maxDeceleration;
    private final double maxLateralAcceleration;

    private final double kS;
    private final double kV;
    private final double kA;

    private final double translationalKp;
    private final double translationalKi;
    private final double translationalKd;

    private final double headingKp;
    private final double headingKi;
    private final double headingKd;

    private final double positionTolerance;
    private final double headingTolerance;
    private final double settleTimeout;
    private final double derivativeFilter;
    private final double integralLimit;

    private final double reactiveErrorScale;
    private final double reactiveAuthority;
    private final double minConfidenceAuthority;
    private final double maxCorrectionVelocity;

    private FollowerConstants(Builder b) {
        this.maxVelocity = b.maxVelocity;
        this.maxAcceleration = b.maxAcceleration;
        this.maxDeceleration = Double.isNaN(b.maxDeceleration)
                ? b.maxAcceleration : b.maxDeceleration;
        this.maxLateralAcceleration = b.maxLateralAcceleration;
        this.kS = b.kS;
        this.kV = b.kV;
        this.kA = b.kA;
        this.translationalKp = b.translationalKp;
        this.translationalKi = b.translationalKi;
        this.translationalKd = b.translationalKd;
        this.headingKp = b.headingKp;
        this.headingKi = b.headingKi;
        this.headingKd = b.headingKd;
        this.positionTolerance = b.positionTolerance;
        this.headingTolerance = b.headingTolerance;
        this.settleTimeout = b.settleTimeout;
        this.derivativeFilter = b.derivativeFilter;
        this.integralLimit = b.integralLimit;
        this.reactiveErrorScale = b.reactiveErrorScale;
        this.reactiveAuthority = b.reactiveAuthority;
        this.minConfidenceAuthority = b.minConfidenceAuthority;
        this.maxCorrectionVelocity = Double.isNaN(b.maxCorrectionVelocity)
                ? b.maxVelocity : b.maxCorrectionVelocity;
    }

    public static Builder builder(DistanceUnit unit) {
        return new Builder(unit);
    }

    public static final class Builder {
        private final DistanceUnit unit;

        private double maxVelocity = Double.NaN;
        private double maxAcceleration = Double.NaN;
        private double maxDeceleration = Double.NaN;
        private double maxLateralAcceleration = Double.POSITIVE_INFINITY;

        private double kS = 0.0;
        private double kV = Double.NaN;
        private double kA = 0.0;

        private double translationalKp = 0.1;
        private double translationalKi = 0.0;
        private double translationalKd = 0.0;

        private double headingKp = 1.0;
        private double headingKi = 0.0;
        private double headingKd = 0.0;

        private double positionTolerance;
        private double headingTolerance = Math.toRadians(2.0);
        private double settleTimeout = 0.5;
        private double derivativeFilter = 0.6;
        private double integralLimit = 0.2;

        private double reactiveErrorScale = 4.0;
        private double reactiveAuthority = 2.5;
        private double minConfidenceAuthority = 0.35;
        private double maxCorrectionVelocity = Double.NaN;

        private Builder(DistanceUnit unit) {
            if (unit == null) {
                throw new IllegalArgumentException("unit must be non-null");
            }
            this.unit = unit;
            this.positionTolerance = DistanceUnit.INCH.toInches(1.0);
        }

        /** Velocity ceiling for the profile, per second, in this builder's unit. */
        public Builder maxVelocity(double v) {
            this.maxVelocity = unit.toInches(v);
            return this;
        }

        /** Acceleration limit, per second squared, in this builder's unit. */
        public Builder maxAcceleration(double a) {
            this.maxAcceleration = unit.toInches(a);
            return this;
        }

        /**
         * Braking limit, per second squared. Defaults to {@code maxAcceleration}
         * if never set.
         */
        public Builder maxDeceleration(double d) {
            this.maxDeceleration = unit.toInches(d);
            return this;
        }

        /**
         * How much sideways acceleration the robot may be asked for while
         * cornering, per second squared, in this builder's unit.
         *
         * <p><b>Off by default</b>, and deliberately so: enabling it slows some
         * existing paths, and the right value has to be measured rather than
         * guessed. Left off, the profile will plan a hairpin at top speed, and
         * the robot will either slide -- at which point the wheels measure
         * something the chassis is not doing, and odometry goes with it -- or
         * fail to turn that tightly and cut the corner.
         *
         * <p>The ceiling is the friction available: roughly {@code mu * g}, so
         * about 230 to 390 in/s^2 for mecanum wheels on tiles depending on how
         * clean they are. Measure yours by driving a circle of known radius,
         * raising speed until the robot starts to slide, then computing
         * {@code v^2 / r} and taking a good margin off it.
         *
         * <p>The follower brakes into a tight corner ahead of time, exactly as
         * it does for a per-segment cap, so the robot arrives already slow
         * enough rather than discovering the corner at speed.
         */
        public Builder maxLateralAcceleration(double a) {
            this.maxLateralAcceleration = unit.toInches(a);
            return this;
        }

        /** Static friction feedforward: the power needed to just start moving. */
        public Builder kS(double kS) {
            this.kS = kS;
            return this;
        }

        /** Velocity feedforward: power per inch per second. */
        public Builder kV(double kV) {
            this.kV = kV;
            return this;
        }

        /** Acceleration feedforward: power per inch per second squared. */
        public Builder kA(double kA) {
            this.kA = kA;
            return this;
        }

        /**
         * Translational correction gains. Error is in inches, output is inches
         * per second of corrective velocity, so kP has units of 1/second.
         */
        public Builder translationalPID(double kP, double kI, double kD) {
            this.translationalKp = kP;
            this.translationalKi = kI;
            this.translationalKd = kD;
            return this;
        }

        /**
         * Heading gains. Error is radians, output is radians per second, so kP
         * is again 1/second.
         */
        public Builder headingPID(double kP, double kI, double kD) {
            this.headingKp = kP;
            this.headingKi = kI;
            this.headingKd = kD;
            return this;
        }

        /** How close counts as arrived, in this builder's unit. */
        public Builder positionTolerance(double tolerance) {
            this.positionTolerance = unit.toInches(tolerance);
            return this;
        }

        /** How close counts as pointed the right way, radians. */
        public Builder headingTolerance(double radians) {
            this.headingTolerance = radians;
            return this;
        }

        /**
         * How long, in seconds, the follower keeps correcting after the profile
         * has run out before giving up and declaring itself done anyway.
         *
         * <p>Without a cap here, a robot pinned an inch short of its target holds
         * the auto hostage for the rest of the match. The follower gives up and
         * moves on; it reports the failure through {@code isAtTarget()} so the
         * OpMode can decide what that means.
         */
        public Builder settleTimeout(double seconds) {
            this.settleTimeout = seconds;
            return this;
        }

        /** Derivative smoothing for both controllers, [0, 1). */
        public Builder derivativeFilter(double filter) {
            this.derivativeFilter = filter;
            return this;
        }

        /** Cap on each integral term's contribution to its controller output. */
        public Builder integralLimit(double limit) {
            this.integralLimit = limit;
            return this;
        }

        /**
         * The position error, in this builder's unit, at which the reactive
         * correction reaches its full authority.
         *
         * <p>This is the blend weight's scale. Below it the follower behaves like
         * a plain profile tracker; approaching it, correction authority ramps up
         * to {@link #reactiveAuthority(double)}. Set it around the largest error
         * you consider "still basically on path" -- a few inches. Setting it very
         * small makes the follower twitchy, since ordinary tracking error then
         * triggers full reactive gain.
         */
        public Builder reactiveErrorScale(double scale) {
            this.reactiveErrorScale = unit.toInches(scale);
            return this;
        }

        /**
         * How much the correction is multiplied by once the error reaches
         * {@link #reactiveErrorScale(double)}. Must be at least 1.
         *
         * <p>1.0 disables reactive blending entirely and leaves a pure
         * profile-plus-PID follower. The default of 2.5 means a badly disturbed
         * robot corrects two and a half times as hard as one that is merely
         * tracking imperfectly.
         */
        public Builder reactiveAuthority(double multiplier) {
            this.reactiveAuthority = multiplier;
            return this;
        }

        /**
         * The floor on how much a low-confidence pose estimate can scale back
         * correction authority, in [0, 1].
         *
         * <p>With a fusing localizer, correction is scaled by how much the
         * estimate is trusted, so the robot does not lunge toward a pose it is
         * unsure of. This floor stops that from disabling correction outright --
         * at 0.35 a completely untrusted estimate still gets 35% authority.
         * Irrelevant for localizers that do not report confidence.
         */
        public Builder minConfidenceAuthority(double floor) {
            this.minConfidenceAuthority = floor;
            return this;
        }

        /**
         * Ceiling on the corrective velocity, in this builder's unit per second.
         * Defaults to {@code maxVelocity}.
         *
         * <p>Without a cap, a large error times full reactive authority asks for
         * a speed the robot cannot produce; the wheel commands then saturate and
         * get scaled down, and the profile's feedforward -- the part that knows
         * where the robot is supposed to be going -- is what gets squeezed out.
         */
        public Builder maxCorrectionVelocity(double velocity) {
            this.maxCorrectionVelocity = unit.toInches(velocity);
            return this;
        }

        public FollowerConstants build() {
            if (Double.isNaN(maxVelocity) || Double.isNaN(maxAcceleration)) {
                throw new IllegalStateException(
                        "maxVelocity and maxAcceleration are required");
            }
            if (Double.isNaN(kV)) {
                throw new IllegalStateException(
                        "kV is required; start from 1 / maxVelocity and tune from there");
            }
            if (reactiveAuthority < 1.0) {
                throw new IllegalStateException(
                        "reactiveAuthority must be at least 1 (1 disables reactive blending); got "
                                + reactiveAuthority);
            }
            if (reactiveErrorScale <= 0) {
                throw new IllegalStateException("reactiveErrorScale must be positive");
            }
            if (maxLateralAcceleration <= 0) {
                throw new IllegalStateException(
                        "maxLateralAcceleration must be positive, or infinite to disable");
            }
            if (minConfidenceAuthority < 0 || minConfidenceAuthority > 1) {
                throw new IllegalStateException(
                        "minConfidenceAuthority must be in [0, 1]; got " + minConfidenceAuthority);
            }
            return new FollowerConstants(this);
        }
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    /**
     * The cornering acceleration limit, inches per second squared, or
     * {@link Double#POSITIVE_INFINITY} when curvature is not limited.
     */
    public double getMaxLateralAcceleration() {
        return maxLateralAcceleration;
    }

    public double getKS() {
        return kS;
    }

    public double getKV() {
        return kV;
    }

    public double getKA() {
        return kA;
    }

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public double getSettleTimeout() {
        return settleTimeout;
    }

    public double getReactiveErrorScale() {
        return reactiveErrorScale;
    }

    public double getReactiveAuthority() {
        return reactiveAuthority;
    }

    public double getMinConfidenceAuthority() {
        return minConfidenceAuthority;
    }

    public double getMaxCorrectionVelocity() {
        return maxCorrectionVelocity;
    }

    /**
     * The factor the translational correction is multiplied by, given how far
     * off the path the robot is and how much the pose estimate is trusted.
     *
     * <p>Lives here rather than in the follower so the blend can be unit-tested
     * as a pure function, and so a team can plot it against error while tuning
     * without instantiating a follower.
     *
     * @param positionError distance from the profile's setpoint, inches
     * @param confidence    pose confidence in [0, 1]
     */
    public double correctionAuthority(double positionError, double confidence) {
        double blend = Math.max(0.0, Math.min(1.0, positionError / reactiveErrorScale));
        double reactive = 1.0 + (reactiveAuthority - 1.0) * blend;

        double trust = Math.max(0.0, Math.min(1.0, confidence));
        double confidenceFactor =
                minConfidenceAuthority + (1.0 - minConfidenceAuthority) * trust;

        return reactive * confidenceFactor;
    }

    /** A fresh translational controller with these gains. */
    public PIDFController newTranslationalController() {
        return new PIDFController(translationalKp, translationalKi, translationalKd,
                integralLimit, derivativeFilter);
    }

    /** A fresh heading controller with these gains. */
    public PIDFController newHeadingController() {
        return new PIDFController(headingKp, headingKi, headingKd,
                integralLimit, derivativeFilter);
    }
}
