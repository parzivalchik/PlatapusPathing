package com.platapus.pathing.tuning;

/**
 * Holds the feedforward gains the follower is currently using, and the ones it
 * will switch to next.
 *
 * <p>The whole point of this class is the gap between those two. Feedforward
 * constants must not change while a path is being followed: the profile was
 * planned against one model, and swapping the model underneath it mid-path makes
 * the robot's response discontinuous at an arbitrary moment. Worse, it makes the
 * failure unreproducible -- the same auto behaves differently depending on how
 * much the tuner happened to learn beforehand.
 *
 * <p>So the tuner {@link #propose}s at any time, and the follower {@link
 * #commitPending()}s only when it starts a new path. The rule is enforced by the
 * structure rather than by remembering to obey it.
 *
 * <p>A proposal is also screened before it can be adopted: implausible fits are
 * refused outright, and a fit that barely differs from what is already active is
 * not worth the disruption of swapping.
 */
public final class TunableFeedforward {

    /** Below this relative change, a new fit is not worth adopting. */
    private static final double MIN_RELATIVE_CHANGE = 0.02;

    private volatile FeedforwardGains active;
    private FeedforwardGains pending;
    private FeedforwardGains lastRejected;
    private int commitCount;

    public TunableFeedforward(FeedforwardGains initial) {
        if (initial == null) {
            throw new IllegalArgumentException("initial gains must be non-null");
        }
        this.active = initial;
    }

    /** The gains in force right now. Safe to call every control loop. */
    public FeedforwardGains get() {
        return active;
    }

    /**
     * Offers a new fit for adoption at the next path boundary. Never changes
     * what {@link #get()} returns.
     *
     * @return true if the proposal was queued, false if it was screened out
     */
    public boolean propose(FeedforwardGains gains) {
        if (gains == null) {
            lastRejected = null;
            return false;
        }
        // Clamp meaningless negative friction terms before judging the fit, so a
        // noise-level negative kS does not discard a good kV along with it.
        FeedforwardGains candidate = gains.sanitized();
        if (!candidate.isPlausible()) {
            lastRejected = gains;
            return false;
        }
        if (candidate.relativeDistanceFrom(active) < MIN_RELATIVE_CHANGE) {
            // Close enough to what is already running that swapping would be
            // churn: it would invalidate a team's feel for the robot without
            // measurably changing anything.
            return false;
        }
        pending = candidate;
        return true;
    }

    /**
     * Adopts any queued proposal. Called by the follower when a path starts --
     * never mid-path.
     *
     * @return true if the active gains actually changed
     */
    public boolean commitPending() {
        if (pending == null) {
            return false;
        }
        active = pending;
        pending = null;
        commitCount++;
        return true;
    }

    /** Whether a proposal is waiting for the next path boundary. */
    public boolean hasPending() {
        return pending != null;
    }

    /** The queued proposal, or null. */
    public FeedforwardGains getPending() {
        return pending;
    }

    /** The most recent proposal refused as implausible, or null. */
    public FeedforwardGains getLastRejected() {
        return lastRejected;
    }

    /** How many times the active gains have been replaced. */
    public int getCommitCount() {
        return commitCount;
    }

    /**
     * Replaces the active gains immediately, bypassing the pending queue.
     *
     * <p>For loading saved constants at OpMode init, before anything is running.
     * Calling it mid-path is exactly what the rest of this class exists to
     * prevent, so do not.
     */
    public void setActiveImmediately(FeedforwardGains gains) {
        if (gains == null) {
            throw new IllegalArgumentException("gains must be non-null");
        }
        this.active = gains;
        this.pending = null;
    }

    @Override
    public String toString() {
        return "TunableFeedforward(active=" + active
                + (pending != null ? ", pending=" + pending : "") + ")";
    }
}
