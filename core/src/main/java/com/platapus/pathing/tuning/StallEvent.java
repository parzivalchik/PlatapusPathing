package com.platapus.pathing.tuning;

/** What the stall detector reports when a motor has been drawing too much for too long. */
public final class StallEvent {

    /** Which motor, in the drivetrain's own wheel order. */
    public final int motorIndex;
    /** Measured current, amps. */
    public final double measuredCurrent;
    /** What the feedforward model predicted it should be drawing, amps. */
    public final double expectedCurrent;
    /** The wheel velocity the follower asked for, inches per second. */
    public final double commandedVelocity;
    /** How long the condition had persisted when this fired, seconds. */
    public final double durationSeconds;
    /** Clock time the event fired, seconds. */
    public final double timestamp;

    public StallEvent(int motorIndex, double measuredCurrent, double expectedCurrent,
                      double commandedVelocity, double durationSeconds, double timestamp) {
        this.motorIndex = motorIndex;
        this.measuredCurrent = measuredCurrent;
        this.expectedCurrent = expectedCurrent;
        this.commandedVelocity = commandedVelocity;
        this.durationSeconds = durationSeconds;
        this.timestamp = timestamp;
    }

    /** How many times more current than predicted is being drawn. */
    public double getCurrentRatio() {
        return Math.abs(expectedCurrent) < 1e-6
                ? Double.POSITIVE_INFINITY
                : measuredCurrent / expectedCurrent;
    }

    @Override
    public String toString() {
        return String.format(
                "StallEvent(motor %d: %.2fA measured vs %.2fA expected, commanded v=%.1f\"/s, %.0fms)",
                motorIndex, measuredCurrent, expectedCurrent, commandedVelocity,
                durationSeconds * 1000);
    }
}
