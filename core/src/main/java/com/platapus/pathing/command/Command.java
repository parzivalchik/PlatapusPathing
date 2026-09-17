package com.platapus.pathing.command;

/**
 * A unit of work that advances a little each control loop.
 *
 * <p>Deliberately minimal, and deliberately <b>not</b> a framework. There is no
 * scheduler singleton here, no subsystem model, no requirement declarations, and
 * no exclusive-resource arbitration. Those are the parts of a command library
 * that force themselves on the rest of your code, and a path-following library
 * has no business imposing them.
 *
 * <p>What this gives you is composition: a way to say "drive this path, then
 * drop the sample, then drive that path" without hand-rolling a state machine
 * with an integer and a switch. If you already use FTCLib or Road Runner's
 * actions, ignore this package entirely -- {@link
 * com.platapus.pathing.follower.Follower} is designed to be driven directly
 * from either, and nothing in the follower knows this package exists.
 *
 * <h2>Lifecycle</h2>
 * <ol>
 *   <li>{@link #initialize()} once, on the first {@code execute} cycle;</li>
 *   <li>{@link #execute()} every loop until {@link #isFinished()} returns true;</li>
 *   <li>{@link #end(boolean)} once, with {@code interrupted} saying whether it
 *       finished on its own terms or was cancelled.</li>
 * </ol>
 *
 * <p>A command must not block. Everything here is cooperative: one call, one
 * loop's worth of progress, then return.
 */
public interface Command {

    /** Called once before the first {@link #execute()}. */
    default void initialize() {
    }

    /** One loop's worth of work. Must return promptly. */
    void execute();

    /** Whether this command is done. Checked after every {@link #execute()}. */
    boolean isFinished();

    /**
     * Called once when the command stops.
     *
     * @param interrupted true if it was cancelled or lost a race, false if
     *                    {@link #isFinished()} returned true on its own
     */
    default void end(boolean interrupted) {
    }

    /** This command, then {@code next}. */
    default Command andThen(Command next) {
        return Commands.sequence(this, next);
    }

    /** This command alongside {@code other}, finishing when both do. */
    default Command alongWith(Command other) {
        return Commands.parallel(this, other);
    }

    /** This command alongside {@code other}, finishing when either does. */
    default Command raceWith(Command other) {
        return Commands.race(this, other);
    }

    /** This command, cancelled if it runs longer than {@code seconds}. */
    default Command withTimeout(double seconds,
                                com.platapus.pathing.control.Clock clock) {
        return Commands.race(this, Commands.waitSeconds(seconds, clock));
    }
}
