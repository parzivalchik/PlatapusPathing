package com.platapus.pathing.follower;

import com.platapus.pathing.control.Clock;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.control.PIDFController;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.kinematics.Kinematics;
import com.platapus.pathing.localization.Localizer;
import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.TunableFeedforward;

/**
 * Holds the robot at a fixed pose, correcting whatever pushes it off.
 *
 * <p>{@link PathFollower} deliberately stops when its profile runs out: an auto
 * that fought to stay on its last waypoint for the rest of the match would be a
 * surprise, not a feature. This is the opposite tool -- parked at a scoring
 * position while a mechanism runs, or braced against a partner robot leaning on
 * you, it keeps correcting until told to stop.
 *
 * <p>It is the follower's correction law with the profile removed. The target
 * velocity is zero, so there is no path feedforward; everything commanded is
 * error correction, scaled by the same reactive authority and pose confidence
 * the follower uses, pushed through the same kinematics.
 *
 * <pre>{@code
 * holder.hold(new Pose2d(-36, -36, Math.toRadians(90)));
 * while (opModeIsActive() && armIsMoving()) {
 *     holder.update();
 * }
 * holder.stop();
 * }</pre>
 *
 * <h2>The deadband</h2>
 * Inside {@code positionTolerance} and {@code headingTolerance} the motors are
 * cut rather than trimmed. A PID asked to hold a stationary robot exactly will
 * hunt around the target forever, which on a real drivetrain is audible buzzing,
 * warm motors and wasted battery for no positional gain. The cost is that the
 * robot may drift up to one tolerance before anything resists -- so set the
 * tolerances to what you actually need, not to the smallest number you can.
 */
public final class PoseHolder {

    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final FollowerConstants constants;
    private final Clock clock;
    private final Kinematics kinematics;
    private final TunableFeedforward feedforward;

    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController headingController;

    private Pose2d target;
    private Pose2d pose = Pose2d.ZERO;
    private boolean holding;
    private double lastUpdateTime = Double.NaN;
    private double lastPositionError;
    private double lastHeadingError;
    private double[] lastPowers = new double[0];

    public PoseHolder(Drivetrain drivetrain, Localizer localizer,
                      FollowerConstants constants) {
        this(drivetrain, localizer, constants, Clock.system());
    }

    /** Test seam: a hand-advanced clock steps this deterministically. */
    public PoseHolder(Drivetrain drivetrain, Localizer localizer,
                      FollowerConstants constants, Clock clock) {
        this(drivetrain, localizer, constants, clock,
                new TunableFeedforward(new FeedforwardGains(
                        constants.getKS(), constants.getKV(), constants.getKA())));
    }

    /**
     * Shares a {@link TunableFeedforward} with a follower, so an online tuner's
     * fit applies here too.
     *
     * <p>Unlike the follower this adopts a pending fit immediately on {@link
     * #hold}, because there is no profile planned against the old model to
     * invalidate.
     */
    public PoseHolder(Drivetrain drivetrain, Localizer localizer,
                      FollowerConstants constants, Clock clock,
                      TunableFeedforward feedforward) {
        if (drivetrain == null || localizer == null || constants == null
                || clock == null || feedforward == null) {
            throw new IllegalArgumentException("all constructor arguments must be non-null");
        }
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.constants = constants;
        this.clock = clock;
        this.kinematics = drivetrain.getKinematics();
        this.feedforward = feedforward;

        this.xController = constants.newTranslationalController();
        this.yController = constants.newTranslationalController();
        this.headingController = constants.newHeadingController();
    }

    /** Begins holding {@code target}, in field coordinates. */
    public void hold(Pose2d target) {
        if (target == null) {
            throw new IllegalArgumentException("target must be non-null");
        }
        this.target = target;
        this.holding = true;
        this.lastUpdateTime = Double.NaN;

        feedforward.commitPending();

        // Stale integral from a previous hold would kick the motors on the first
        // loop of this one, which is exactly what a hold must not do.
        xController.reset();
        yController.reset();
        headingController.reset();
    }

    /** Holds wherever the robot currently is. */
    public void holdCurrentPose() {
        localizer.update();
        hold(localizer.getPose());
    }

    /** One correction iteration. Call every control loop while holding. */
    public void update() {
        double now = clock.seconds();
        double dt = Double.isNaN(lastUpdateTime) ? 0.0 : now - lastUpdateTime;
        lastUpdateTime = now;

        localizer.update();
        pose = localizer.getPose();

        if (!holding) {
            return;
        }

        Vector2d positionError = target.position.minus(pose.position);
        double headingError = pose.headingErrorTo(target.heading);
        lastPositionError = positionError.norm();
        lastHeadingError = Math.abs(headingError);

        boolean withinPosition = lastPositionError <= constants.getPositionTolerance();
        boolean withinHeading = lastHeadingError <= constants.getHeadingTolerance();

        if (withinPosition && withinHeading) {
            // Close enough. Cut power rather than hunt -- see the class note.
            xController.reset();
            yController.reset();
            headingController.reset();
            lastPowers = new double[kinematics.getWheelCount()];
            drivetrain.setWheelPowers(lastPowers);
            return;
        }

        double authority = constants.correctionAuthority(
                lastPositionError, localizer.getConfidence());

        Vector2d correction = new Vector2d(
                xController.calculate(positionError.x, dt),
                yController.calculate(positionError.y, dt)).times(authority);

        double cap = constants.getMaxCorrectionVelocity();
        if (correction.norm() > cap) {
            correction = correction.times(cap / correction.norm());
        }

        double omega = headingController.calculate(headingError, dt);

        // No path feedforward: the target is stationary, so every inch per
        // second commanded here is correction.
        ChassisSpeeds robotSpeeds =
                ChassisSpeeds.fromFieldRelative(correction, omega, pose.heading);

        lastPowers = toPowers(kinematics.toWheelVelocities(robotSpeeds));
        drivetrain.setWheelPowers(lastPowers);
    }

    /**
     * Applies the feedforward model per wheel and normalises into motor powers.
     *
     * <p>Same shape as the follower's, minus the acceleration term: holding
     * commands a velocity to close an error, never a planned acceleration.
     */
    private double[] toPowers(double[] wheelVelocities) {
        FeedforwardGains gains = feedforward.get();
        double deadband = 1e-3;

        double[] powers = new double[wheelVelocities.length];
        double maxMagnitude = 0.0;
        for (int i = 0; i < wheelVelocities.length; i++) {
            double v = wheelVelocities[i];
            double staticTerm = Math.abs(v) > deadband ? Math.signum(v) * gains.kS : 0.0;
            powers[i] = staticTerm + gains.kV * v;
            maxMagnitude = Math.max(maxMagnitude, Math.abs(powers[i]));
        }

        // Scale rather than clip, so a saturated command keeps its direction.
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxMagnitude;
            }
        }
        return powers;
    }

    /** Stops holding and cuts the motors. */
    public void stop() {
        holding = false;
        target = null;
        lastPowers = new double[kinematics.getWheelCount()];
        drivetrain.stop();
    }

    /** Whether a hold is active. */
    public boolean isHolding() {
        return holding;
    }

    /** Whether the robot is within both tolerances of the target right now. */
    public boolean isAtTarget() {
        return holding
                && lastPositionError <= constants.getPositionTolerance()
                && lastHeadingError <= constants.getHeadingTolerance();
    }

    /** The pose being held, or null when not holding. */
    public Pose2d getTarget() {
        return target;
    }

    /** The most recent pose estimate. */
    public Pose2d getPose() {
        return pose;
    }

    /** Distance from the held target, inches. */
    public double getPositionError() {
        return lastPositionError;
    }

    /** Absolute heading error against the held target, radians. */
    public double getHeadingError() {
        return lastHeadingError;
    }

    /** The powers written on the most recent loop. */
    public double[] getLastPowers() {
        return lastPowers.clone();
    }

    /** The feedforward model in use, shareable with a follower and a tuner. */
    public TunableFeedforward getFeedforward() {
        return feedforward;
    }
}
