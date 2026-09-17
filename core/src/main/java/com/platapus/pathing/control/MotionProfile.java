package com.platapus.pathing.control;

/**
 * A trapezoidal motion profile over a fixed distance, starting and ending at
 * rest.
 *
 * <p>The profile is the "time-consistent" half of the follower: it decides where
 * the robot is supposed to be at every instant, independently of where it
 * actually is. That makes an auto repeatable -- the same path takes the same
 * number of seconds every run -- which is what you want when the rest of the auto
 * is timed around it.
 *
 * <p>When the distance is too short to reach the cruise velocity the profile
 * degenerates to a triangle, accelerating to whatever peak speed still leaves
 * room to stop.
 */
public final class MotionProfile {

    private final double distance;
    private final double cruiseVelocity;
    private final double acceleration;
    private final double deceleration;

    private final double accelTime;
    private final double cruiseTime;
    private final double decelTime;
    private final double accelDistance;
    private final double cruiseDistance;
    private final double duration;
    private final double peakVelocity;

    /**
     * @param distance       path length to cover, inches. Must be non-negative.
     * @param cruiseVelocity velocity ceiling, inches per second
     * @param acceleration   inches per second squared, positive
     * @param deceleration   inches per second squared, positive. Separate from
     *                       acceleration because a robot can almost always brake
     *                       harder than it can accelerate, and profiling both at
     *                       the (lower) accel limit wastes real time on every path.
     */
    public MotionProfile(double distance, double cruiseVelocity, double acceleration,
                         double deceleration) {
        if (distance < 0) {
            throw new IllegalArgumentException("distance must be non-negative, got " + distance);
        }
        if (cruiseVelocity <= 0 || acceleration <= 0 || deceleration <= 0) {
            throw new IllegalArgumentException(
                    "cruiseVelocity, acceleration and deceleration must all be positive");
        }

        this.distance = distance;
        this.acceleration = acceleration;
        this.deceleration = deceleration;

        double accelDist = cruiseVelocity * cruiseVelocity / (2.0 * acceleration);
        double decelDist = cruiseVelocity * cruiseVelocity / (2.0 * deceleration);

        if (accelDist + decelDist <= distance) {
            // Full trapezoid: there is room to reach cruise speed and hold it.
            this.peakVelocity = cruiseVelocity;
            this.accelDistance = accelDist;
            this.cruiseDistance = distance - accelDist - decelDist;
        } else {
            // Triangle: solve for the peak speed that exactly fits, by setting
            // the accel and decel distances to sum to the total.
            this.peakVelocity = Math.sqrt(
                    2.0 * distance * acceleration * deceleration / (acceleration + deceleration));
            this.accelDistance = peakVelocity * peakVelocity / (2.0 * acceleration);
            this.cruiseDistance = 0.0;
        }

        this.cruiseVelocity = peakVelocity;
        this.accelTime = peakVelocity / acceleration;
        this.cruiseTime = peakVelocity < 1e-9 ? 0.0 : cruiseDistance / peakVelocity;
        this.decelTime = peakVelocity / deceleration;
        this.duration = accelTime + cruiseTime + decelTime;
    }

    /** The profile state at {@code time} seconds after the start. */
    public MotionState get(double time) {
        if (time <= 0) {
            return new MotionState(0.0, 0.0, distance > 0 ? acceleration : 0.0);
        }
        if (time >= duration) {
            return new MotionState(distance, 0.0, 0.0);
        }

        if (time < accelTime) {
            return new MotionState(
                    0.5 * acceleration * time * time,
                    acceleration * time,
                    acceleration);
        }

        double afterAccel = time - accelTime;
        if (afterAccel < cruiseTime) {
            return new MotionState(
                    accelDistance + cruiseVelocity * afterAccel,
                    cruiseVelocity,
                    0.0);
        }

        double intoDecel = afterAccel - cruiseTime;
        double v = cruiseVelocity - deceleration * intoDecel;
        double s = accelDistance + cruiseDistance
                + cruiseVelocity * intoDecel - 0.5 * deceleration * intoDecel * intoDecel;
        return new MotionState(Math.min(s, distance), Math.max(v, 0.0), -deceleration);
    }

    /** Total time the profile takes, seconds. */
    public double duration() {
        return duration;
    }

    /** Total distance the profile covers, inches. */
    public double distance() {
        return distance;
    }

    /** The highest speed this profile actually reaches, inches per second. */
    public double peakVelocity() {
        return peakVelocity;
    }

    /** Whether this profile is a triangle -- too short to reach cruise speed. */
    public boolean isTriangular() {
        return cruiseDistance <= 1e-9;
    }

    @Override
    public String toString() {
        return String.format("MotionProfile(%.2f\" in %.2fs, peak %.1f\"/s%s)",
                distance, duration, peakVelocity, isTriangular() ? ", triangular" : "");
    }
}
