package com.platapus.pathing.tuning;

/**
 * Somewhere to keep fitted constants between OpMode runs.
 *
 * <p>Without persistence the tuner relearns the robot from scratch every time
 * the OpMode starts, which means the first autonomous of every match runs on
 * whatever guess was compiled in. Saving the fit means the second match starts
 * from the first match's answer.
 */
public interface TuningStore {

    /** The saved gains, or {@code null} if nothing has been saved yet. */
    FeedforwardGains load();

    /**
     * Saves {@code gains}, replacing anything already stored.
     *
     * <p>Implementations must not throw on failure -- a Control Hub with a full
     * or read-only filesystem is a nuisance, not a reason to abort an auto.
     *
     * @return true if the save succeeded
     */
    boolean save(FeedforwardGains gains);
}
