package com.platapus.pathing.command;

import com.platapus.pathing.follower.PoseHolder;
import com.platapus.pathing.geometry.Pose2d;

import java.util.function.BooleanSupplier;

/**
 * Holds a pose until something else says to stop.
 *
 * <p>A hold has no natural end -- that is the point of it -- so this never
 * finishes on its own. Give it an explicit condition, or run it as the losing
 * side of a {@link Commands#deadline} while the real work happens:
 *
 * <pre>{@code
 * Commands.deadline(
 *         Commands.sequence(              // the work that decides how long
 *                 Commands.run(arm::raise),
 *                 Commands.waitUntil(arm::atTop),
 *                 Commands.run(claw::open)),
 *         new HoldPositionCommand(holder, scoringPose))   // held throughout
 * }</pre>
 *
 * <p>Used bare inside {@link Commands#sequence} with no end condition, it would
 * hold forever and the sequence would never advance.
 */
public final class HoldPositionCommand implements Command {

    private final PoseHolder holder;
    private final Pose2d target;
    private final BooleanSupplier until;
    /** When true, the pose to hold is decided at initialize(), not now. */
    private final boolean captureOnStart;

    /** Holds {@code target} indefinitely. Only sensible inside a race or deadline. */
    public HoldPositionCommand(PoseHolder holder, Pose2d target) {
        this(holder, target, () -> false);
    }

    /** Holds {@code target} until {@code until} returns true. */
    public HoldPositionCommand(PoseHolder holder, Pose2d target, BooleanSupplier until) {
        this(holder, target, until, false);
    }

    private HoldPositionCommand(PoseHolder holder, Pose2d target,
                                BooleanSupplier until, boolean captureOnStart) {
        Commands.requireNonNull(holder, "holder");
        Commands.requireNonNull(until, "until");
        if (!captureOnStart) {
            Commands.requireNonNull(target, "target");
        }
        this.holder = holder;
        this.target = target;
        this.until = until;
        this.captureOnStart = captureOnStart;
    }

    /**
     * Holds wherever the robot is when this starts, rather than a pose fixed in
     * advance -- useful straight after a path, where the exact stopping point is
     * whatever the follower achieved.
     */
    public static HoldPositionCommand holdCurrentPose(PoseHolder holder,
                                                      BooleanSupplier until) {
        return new HoldPositionCommand(holder, null, until, true);
    }

    @Override
    public void initialize() {
        if (captureOnStart) {
            holder.holdCurrentPose();
        } else {
            holder.hold(target);
        }
    }

    @Override
    public void execute() {
        holder.update();
    }

    @Override
    public boolean isFinished() {
        return until.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        // Either way the hold is over, and leaving it running would fight
        // whatever command comes next for the same motors.
        holder.stop();
    }
}
