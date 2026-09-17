package com.platapus.pathing.follower;

import com.platapus.pathing.control.Clock;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.control.ConstrainedProfile;
import com.platapus.pathing.control.MotionState;
import com.platapus.pathing.control.PIDFController;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.geometry.Angles;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.kinematics.Kinematics;
import com.platapus.pathing.localization.Localizer;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.PathState;
import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.TunableFeedforward;

/**
 * A hybrid follower: a time-parameterised motion profile with per-wheel
 * feedforward, PID correction back onto the profile's setpoint, and an
 * error-scaled reactive term on top.
 *
 * <p>Each loop it asks the profile where the robot ought to be by now, converts
 * that into a target point and velocity on the path, adds a PID term proportional
 * to how far off the robot actually is, and pushes the result through the
 * drivetrain's kinematics. Because the setpoint advances on a clock rather than
 * on measured progress, a path takes the same time every run -- and a robot that
 * gets shoved does not lose its place in the profile, it just develops an error
 * the PID then works off.
 *
 * <p>On top of that sits the reactive term: correction authority scales with the
 * magnitude of the pose error, so the follower stays a tight profile tracker
 * while it is on-path and becomes aggressive once it is knocked well off. See
 * {@link #computeCorrection}.
 */
public final class PathFollower implements Follower {

    private enum State {
        IDLE,
        FOLLOWING,
        SETTLING
    }

    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final FollowerConstants constants;
    private final Clock clock;
    private final Kinematics kinematics;

    private final TunableFeedforward feedforward;

    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController headingController;

    private State state = State.IDLE;
    private PathChain path;
    private ConstrainedProfile profile;

    private double pathStartTime;
    private double lastUpdateTime = Double.NaN;
    private double settleStartTime;

    private Pose2d pose = Pose2d.ZERO;
    private PathState targetState;
    private PathState projectedState;
    private int currentSegment;
    private double currentT;
    private double lastPositionError;
    private double lastHeadingError;
    private double[] lastPowers = new double[0];
    private double[] lastWheelVelocities = new double[0];
    private double lastConfidence = 1.0;
    private double lastAuthority = 1.0;

    public PathFollower(Drivetrain drivetrain, Localizer localizer,
                        FollowerConstants constants) {
        this(drivetrain, localizer, constants, Clock.system());
    }

    /** Test seam: supply a hand-advanced clock to step the follower deterministically. */
    public PathFollower(Drivetrain drivetrain, Localizer localizer,
                        FollowerConstants constants, Clock clock) {
        this(drivetrain, localizer, constants, clock,
                new TunableFeedforward(new FeedforwardGains(
                        constants.getKS(), constants.getKV(), constants.getKA())));
    }

    /**
     * Drives the follower from a {@link TunableFeedforward} rather than from the
     * constants' fixed gains, so an online tuner can improve them between paths.
     *
     * <p>The follower adopts a pending fit only in {@link #followPath}, never
     * during one. Swapping the model mid-path would make the robot's response
     * discontinuous at an arbitrary moment and the resulting miss impossible to
     * reproduce.
     */
    public PathFollower(Drivetrain drivetrain, Localizer localizer,
                        FollowerConstants constants, Clock clock,
                        TunableFeedforward feedforward) {
        if (drivetrain == null || localizer == null || constants == null || clock == null
                || feedforward == null) {
            throw new IllegalArgumentException("all constructor arguments must be non-null");
        }
        this.feedforward = feedforward;
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.constants = constants;
        this.clock = clock;
        this.kinematics = drivetrain.getKinematics();

        this.xController = constants.newTranslationalController();
        this.yController = constants.newTranslationalController();
        this.headingController = constants.newHeadingController();
    }

    @Override
    public void followPath(PathChain path) {
        if (path == null) {
            throw new IllegalArgumentException("path must be non-null");
        }

        // The one moment it is safe to change the feedforward model: nothing is
        // in flight, and the profile about to be built will be planned against
        // whatever we adopt here.
        feedforward.commitPending();

        this.path = path;
        // Sampling the chain's own per-segment caps and what its curvature
        // physically allows. With no overrides and no lateral limit this is a
        // constant ceiling and the result is the same trapezoid as before.
        final PathChain chainForLimit = path;
        final double globalMax = constants.getMaxVelocity();
        final double lateral = constants.getMaxLateralAcceleration();
        this.profile = new ConstrainedProfile(
                path.length(),
                s -> chainForLimit.speedLimitAtArcLength(s, globalMax, lateral),
                constants.getMaxAcceleration(), constants.getMaxDeceleration());

        this.pathStartTime = clock.seconds();
        this.lastUpdateTime = Double.NaN;
        this.currentSegment = 0;
        this.currentT = 0.0;
        this.targetState = path.startState();
        this.projectedState = this.targetState;
        this.state = State.FOLLOWING;

        // Stale integral or derivative from the previous path would otherwise
        // kick the motors on the very first loop of this one.
        xController.reset();
        yController.reset();
        headingController.reset();
    }

    @Override
    public void update() {
        double now = clock.seconds();
        double dt = Double.isNaN(lastUpdateTime) ? 0.0 : now - lastUpdateTime;
        lastUpdateTime = now;

        localizer.update();
        pose = localizer.getPose();

        if (state == State.IDLE) {
            return;
        }

        double elapsed = now - pathStartTime;
        MotionState motion = profile.get(elapsed);

        // Where the profile says we should be.
        targetState = path.stateAtArcLength(motion.position);

        // Where we actually are, projected onto the path. Only used for
        // telemetry and for keeping the projection window moving; the control
        // law tracks the profile, not the projection.
        projectedState = path.project(pose.position, currentSegment, currentT);
        currentSegment = projectedState.segmentIndex;
        currentT = projectedState.t;

        Vector2d positionError = targetState.point.minus(pose.position);
        double headingError = pose.headingErrorTo(targetState.heading);
        lastPositionError = positionError.norm();
        lastHeadingError = Math.abs(headingError);
        lastConfidence = localizer.getConfidence();
        lastAuthority = constants.correctionAuthority(lastPositionError, lastConfidence);

        if (state == State.FOLLOWING && elapsed >= profile.duration()) {
            state = State.SETTLING;
            settleStartTime = now;
        }

        if (state == State.SETTLING) {
            boolean arrived = lastPositionError <= constants.getPositionTolerance()
                    && lastHeadingError <= constants.getHeadingTolerance();
            boolean outOfTime = now - settleStartTime >= constants.getSettleTimeout();
            if (arrived || outOfTime) {
                state = State.IDLE;
                drivetrain.stop();
                return;
            }
        }

        // Feedforward: drive along the path tangent at the profiled speed.
        Vector2d feedforwardVelocity = targetState.tangent.times(motion.velocity);
        Vector2d correctionVelocity = computeCorrection(positionError, dt);
        Vector2d worldVelocity = feedforwardVelocity.plus(correctionVelocity);

        double omega = headingFeedforward(motion) + headingController.calculate(headingError, dt);

        ChassisSpeeds robotSpeeds =
                ChassisSpeeds.fromFieldRelative(worldVelocity, omega, pose.heading);

        // Acceleration feedforward, expressed along the tangent.
        Vector2d worldAcceleration = targetState.tangent.times(motion.acceleration);
        ChassisSpeeds robotAccelerations =
                ChassisSpeeds.fromFieldRelative(worldAcceleration, 0.0, pose.heading);

        double[] wheelVelocities = kinematics.toWheelVelocities(robotSpeeds);
        double[] wheelAccelerations = kinematics.toWheelAccelerations(robotAccelerations);

        lastWheelVelocities = wheelVelocities;
        lastPowers = toPowers(wheelVelocities, wheelAccelerations);
        drivetrain.setWheelPowers(lastPowers);
    }

    /**
     * The corrective velocity that pulls the robot back onto the profile's
     * setpoint, in field coordinates.
     *
     * <p>This is the hybrid part. The PID underneath is the same one a pure
     * profile follower would use, and while the robot is tracking well that is
     * all this returns -- authority sits near 1 and the behaviour is Road
     * Runner-ish: smooth, time-consistent, gentle. As the error grows past
     * {@code reactiveErrorScale} the authority ramps toward {@code
     * reactiveAuthority}, and the follower starts behaving the way Pedro does
     * when it is knocked off: correcting hard and immediately.
     *
     * <p>Blending on error magnitude rather than switching on a threshold is
     * deliberate. A hard switch produces a discontinuity in commanded velocity
     * exactly at the error where the robot is already struggling, and a robot
     * hovering near that threshold chatters between two control regimes.
     *
     * <p>The result is capped, because an uncapped correction at full authority
     * can demand more speed than the drivetrain has, and the normalisation that
     * follows would then eat into the feedforward.
     */
    private Vector2d computeCorrection(Vector2d positionError, double dt) {
        Vector2d pid = new Vector2d(
                xController.calculate(positionError.x, dt),
                yController.calculate(positionError.y, dt));

        Vector2d scaled = pid.times(lastAuthority);

        double cap = constants.getMaxCorrectionVelocity();
        double magnitude = scaled.norm();
        return magnitude > cap ? scaled.times(cap / magnitude) : scaled;
    }

    /**
     * Angular velocity implied by the heading interpolator itself, so the robot
     * turns along with a sweeping heading rather than always lagging it and
     * relying on the P term to catch up.
     *
     * <p>Estimated by sampling the desired heading a short distance further along
     * the path and differencing. Sampling forward by distance rather than by time
     * keeps the estimate stable when the loop rate wobbles.
     */
    private double headingFeedforward(MotionState motion) {
        if (motion.velocity < 1e-6) {
            return 0.0;
        }
        double lookahead = 0.25; // inches
        double aheadPosition = Math.min(motion.position + lookahead, path.length());
        double actualStep = aheadPosition - motion.position;
        if (actualStep < 1e-9) {
            return 0.0;
        }
        double aheadHeading = path.stateAtArcLength(aheadPosition).heading;
        double dHeading = Angles.normalize(aheadHeading - targetState.heading);
        // d(heading)/dt = d(heading)/ds * ds/dt
        return dHeading / actualStep * motion.velocity;
    }

    /**
     * Applies the feedforward model per wheel and normalises the result into
     * motor powers.
     *
     * <p>kS is applied against the sign of the commanded velocity, but only once
     * that velocity is meaningfully non-zero. Applying it at every infinitesimal
     * command makes a stationary robot buzz between +kS and -kS as the sign of a
     * near-zero number flickers.
     */
    private double[] toPowers(double[] wheelVelocities, double[] wheelAccelerations) {
        FeedforwardGains gains = feedforward.get();
        double kS = gains.kS;
        double kV = gains.kV;
        double kA = gains.kA;
        double deadband = 1e-3; // inches per second

        double[] powers = new double[wheelVelocities.length];
        double maxMagnitude = 0.0;
        for (int i = 0; i < wheelVelocities.length; i++) {
            double v = wheelVelocities[i];
            double a = i < wheelAccelerations.length ? wheelAccelerations[i] : 0.0;
            double staticTerm = Math.abs(v) > deadband ? Math.signum(v) * kS : 0.0;
            powers[i] = staticTerm + kV * v + kA * a;
            maxMagnitude = Math.max(maxMagnitude, Math.abs(powers[i]));
        }

        // Scale the whole set rather than clipping wheel by wheel: clipping one
        // wheel changes the ratios and therefore the direction the robot goes.
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxMagnitude;
            }
        }
        return powers;
    }

    @Override
    public boolean isBusy() {
        return state != State.IDLE;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void breakFollowing() {
        state = State.IDLE;
        drivetrain.stop();
    }

    /**
     * Whether the robot actually reached the end of the last path, within
     * tolerance. Distinct from {@code !isBusy()}: the follower also stops when
     * the settle timeout expires, and that case is a miss, not a success.
     */
    public boolean isAtTarget() {
        return lastPositionError <= constants.getPositionTolerance()
                && lastHeadingError <= constants.getHeadingTolerance();
    }

    /** The point on the path the profile currently wants the robot at. */
    public PathState getTargetState() {
        return targetState;
    }

    /** The robot's current pose projected onto the path. */
    public PathState getProjectedState() {
        return projectedState;
    }

    /** Distance from the robot to the profile's current setpoint, inches. */
    public double getPositionError() {
        return lastPositionError;
    }

    /** Absolute heading error against the current setpoint, radians. */
    public double getHeadingError() {
        return lastHeadingError;
    }

    /**
     * The correction multiplier applied on the most recent loop: 1 when the
     * robot is on-path and trusted, rising toward the configured reactive
     * authority as it is knocked off. Worth putting on telemetry while tuning
     * the blend -- it is the single number that says which regime the follower
     * is currently in.
     */
    public double getCorrectionAuthority() {
        return lastAuthority;
    }

    /** The pose confidence the localizer reported on the most recent loop. */
    public double getPoseConfidence() {
        return lastConfidence;
    }

    /** How far along the chain the robot has actually got, inches. */
    public double getArcLengthTravelled() {
        return projectedState == null ? 0.0 : projectedState.arcLength;
    }

    /**
     * The feedforward model this follower uses. Hand it to a {@link
     * com.platapus.pathing.tuning.TuningSession} to have it fitted online.
     */
    public TunableFeedforward getFeedforward() {
        return feedforward;
    }

    /** The powers written to the drivetrain on the most recent loop. */
    public double[] getLastPowers() {
        return lastPowers.clone();
    }

    /**
     * The wheel velocities the follower asked for on the most recent loop,
     * inches per second.
     *
     * <p>These are <b>commanded</b>, not measured. That distinction is what makes
     * {@link com.platapus.pathing.tuning.StallDetector} work: predicting a
     * motor's current from its measured speed is self-fulfilling, since a stalled
     * wheel reads zero and the model then predicts exactly the large current it
     * is really drawing.
     */
    public double[] getLastWheelVelocities() {
        return lastWheelVelocities.clone();
    }

    /** The profile driving the current path, or null when idle. */
    public ConstrainedProfile getProfile() {
        return profile;
    }
}
