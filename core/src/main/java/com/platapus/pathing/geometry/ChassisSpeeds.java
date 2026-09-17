package com.platapus.pathing.geometry;

/**
 * A robot-relative velocity command: forward, left, and yaw rate.
 *
 * <p>{@code vx} and {@code vy} are inches per second, {@code omega} is radians
 * per second CCW. +x is out the front of the robot, +y out its left side.
 */
public final class ChassisSpeeds {

    public static final ChassisSpeeds ZERO = new ChassisSpeeds(0, 0, 0);

    public final double vx;
    public final double vy;
    public final double omega;

    public ChassisSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    /**
     * Converts a velocity expressed in field coordinates into a robot-relative
     * command, given the robot's current heading.
     */
    public static ChassisSpeeds fromFieldRelative(Vector2d worldVelocity, double omega,
                                                  double robotHeading) {
        Vector2d v = worldVelocity.rotated(-robotHeading);
        return new ChassisSpeeds(v.x, v.y, omega);
    }

    public Vector2d translation() {
        return new Vector2d(vx, vy);
    }

    public ChassisSpeeds plus(ChassisSpeeds o) {
        return new ChassisSpeeds(vx + o.vx, vy + o.vy, omega + o.omega);
    }

    public ChassisSpeeds times(double s) {
        return new ChassisSpeeds(vx * s, vy * s, omega * s);
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(vx=%.2f, vy=%.2f, omega=%.2f)", vx, vy, omega);
    }
}
