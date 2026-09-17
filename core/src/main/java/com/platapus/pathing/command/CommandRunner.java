package com.platapus.pathing.command;

/**
 * Drives one root command to completion.
 *
 * <p>This is as close to a scheduler as the library gets, and it is intentionally
 * not one: it owns a single command, has no registry, no requirements, and no
 * opinion about what else is running. Compose everything you want to happen into
 * one tree with {@link Commands}, hand it here, and call {@link #run()} each
 * loop.
 *
 * <pre>{@code
 * CommandRunner runner = new CommandRunner(
 *         Commands.sequence(
 *                 new FollowPathCommand(follower, toBasket),
 *                 Commands.run(arm::raise),
 *                 Commands.waitSeconds(0.4, clock),
 *                 Commands.run(claw::open),
 *                 new FollowPathCommand(follower, toPark)));
 *
 * while (opModeIsActive() && !runner.isFinished()) {
 *     runner.run();
 *     telemetry.update();
 * }
 * }</pre>
 */
public final class CommandRunner {

    private final Command command;
    private boolean initialized;
    private boolean finished;

    public CommandRunner(Command command) {
        Commands.requireNonNull(command, "command");
        this.command = command;
    }

    /**
     * Advances the command by one loop. Safe to keep calling after it has
     * finished; it becomes a no-op rather than restarting anything.
     */
    public void run() {
        if (finished) {
            return;
        }
        if (!initialized) {
            command.initialize();
            initialized = true;
        }

        command.execute();

        if (command.isFinished()) {
            command.end(false);
            finished = true;
        }
    }

    public boolean isFinished() {
        return finished;
    }

    public boolean isRunning() {
        return initialized && !finished;
    }

    /**
     * Stops the command where it is, giving it a chance to clean up.
     *
     * <p>Whether that actually stops the robot is up to the commands involved --
     * {@link FollowPathCommand} does stop the drivetrain, but a command that
     * spun up a shooter has to switch it off in its own {@code end}. The runner
     * has no way to know what needs undoing.
     */
    public void cancel() {
        if (initialized && !finished) {
            command.end(true);
        }
        finished = true;
    }

    /** The command this runner owns. */
    public Command getCommand() {
        return command;
    }
}
