package com.platapus.pathing.control;

/** A point on a motion profile: how far along, how fast, and accelerating how hard. */
public final class MotionState {

    /** Distance along the path, inches. */
    public final double position;
    /** Speed along the path, inches per second. Never negative. */
    public final double velocity;
    /** Acceleration along the path, inches per second squared. */
    public final double acceleration;

    public MotionState(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    @Override
    public String toString() {
        return String.format("MotionState(s=%.2f\", v=%.2f\"/s, a=%.2f\"/s^2)",
                position, velocity, acceleration);
    }
}
