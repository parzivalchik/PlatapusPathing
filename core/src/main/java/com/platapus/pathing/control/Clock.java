package com.platapus.pathing.control;

/**
 * Source of monotonic time, in seconds.
 *
 * <p>Exists so the follower can be stepped deterministically in a unit test. On
 * a robot this is {@link #system()}; in a test it is a counter the test advances
 * by hand, which is what lets the whole follower be exercised without a device
 * and without any real time passing.
 */
public interface Clock {

    /** Seconds since some arbitrary fixed origin. Must never go backwards. */
    double seconds();

    /** Wall-clock time from {@link System#nanoTime()}. */
    static Clock system() {
        final long origin = System.nanoTime();
        return () -> (System.nanoTime() - origin) * 1e-9;
    }
}
