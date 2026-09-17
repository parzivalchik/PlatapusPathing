package com.platapus.pathing.tuning;

/**
 * Notified when a motor starts or stops stalling.
 *
 * <p>The library deliberately does nothing in response to a stall. Whether the
 * right answer is to abort the path, back off and retry, cut power to protect
 * the motor, or carry on regardless depends entirely on the mechanism and the
 * game -- a drivetrain pinned against a wall during a scoring cycle is a very
 * different problem from one wedged under the truss with twenty seconds left.
 * Hard-coding a response would be guessing on the team's behalf.
 *
 * <p>So this reports, and the OpMode or the command layer decides.
 */
public interface StallListener {

    /** A motor has been drawing more current than the model predicts, for long enough to count. */
    void onStallDetected(StallEvent event);

    /** A previously stalled motor is behaving normally again. */
    default void onStallCleared(int motorIndex, double timestamp) {
    }
}
