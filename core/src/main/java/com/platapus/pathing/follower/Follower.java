package com.platapus.pathing.follower;

import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.paths.PathChain;

/**
 * Drives the robot along a {@link PathChain}.
 *
 * <p>Headless by design. There is no scheduler here, no action system, no
 * exclusive-resource model, and no assumption that you are using one. The
 * follower does exactly one thing per {@code update()} call and tells you whether
 * it is finished; how you sequence that against the rest of your auto is yours to
 * decide. It composes with FTCLib commands, with Road Runner actions, with a
 * hand-rolled state machine, or with a plain {@code while} loop.
 *
 * <p>The expected shape of an auto:
 *
 * <pre>{@code
 * follower.followPath(chain);
 * while (opModeIsActive() && follower.isBusy()) {
 *     follower.update();
 *     telemetry.addData("pose", follower.getPose());
 *     telemetry.update();
 * }
 * }</pre>
 */
public interface Follower {

    /**
     * Begins following {@code path}. Replaces whatever was being followed. The
     * robot does not move until {@link #update()} is called.
     */
    void followPath(PathChain path);

    /**
     * Runs one control iteration: read the pose, work out what the motors should
     * do, and command them. Call this as fast as your loop allows -- 50 Hz or
     * better is normal, and the follower measures its own timestep so an uneven
     * loop rate does not corrupt the derivative or integral terms.
     */
    void update();

    /** Whether a path is still being followed. */
    boolean isBusy();

    /** The current pose estimate, in field coordinates. */
    Pose2d getPose();

    /** Abandons the current path and stops the motors. */
    void breakFollowing();
}
