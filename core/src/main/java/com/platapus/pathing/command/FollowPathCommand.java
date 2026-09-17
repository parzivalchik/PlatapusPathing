package com.platapus.pathing.command;

import com.platapus.pathing.follower.Follower;
import com.platapus.pathing.paths.PathChain;

/**
 * Follows a {@link PathChain} as a {@link Command}.
 *
 * <p>The whole adapter, and the only place the command package touches the
 * follower. That the coupling is this thin is the point: the follower has no
 * idea this exists, so a team using FTCLib or Road Runner actions writes their
 * own eight-line equivalent against the same three methods.
 */
public final class FollowPathCommand implements Command {

    private final Follower follower;
    private final PathChain path;

    public FollowPathCommand(Follower follower, PathChain path) {
        Commands.requireNonNull(follower, "follower");
        Commands.requireNonNull(path, "path");
        this.follower = follower;
        this.path = path;
    }

    @Override
    public void initialize() {
        follower.followPath(path);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Losing a race or being cancelled has to stop the drivetrain.
            // Leaving the follower running would keep commanding motors while
            // whatever came next also tries to.
            follower.breakFollowing();
        }
    }
}
