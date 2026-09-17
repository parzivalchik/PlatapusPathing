package com.platapus.pathing.geometry;

/**
 * An incremental robot-relative movement over one loop: how far the robot moved
 * forward and left, and how much it rotated, in its own frame at the start of the
 * interval.
 *
 * <p>This is what a {@code Localizer} reports each cycle and what the pose fusion
 * layer integrates. Distances are inches, rotation is radians CCW.
 */
public final class Twist2d {

    public static final Twist2d ZERO = new Twist2d(0, 0, 0);

    public final double dx;
    public final double dy;
    public final double dTheta;

    public Twist2d(double dx, double dy, double dTheta) {
        this.dx = dx;
        this.dy = dy;
        this.dTheta = dTheta;
    }

    /**
     * Applies this twist to {@code pose} as a constant-curvature arc, and returns
     * the resulting pose.
     *
     * <p>The naive integration -- rotate the delta by the starting heading and add
     * -- assumes the robot drove a straight line then spun in place, which drifts
     * badly on curved paths at low loop rates. The exact solution for a body
     * moving with constant linear and angular velocity uses the arc's chord
     * instead; that is what this computes. As {@code dTheta} approaches zero the
     * closed form becomes numerically unstable, so a Taylor expansion takes over.
     */
    public Pose2d applyTo(Pose2d pose) {
        double sinT;
        double cosT;
        if (Math.abs(dTheta) < 1e-9) {
            // sin(t)/t and (1 - cos(t))/t, expanded around 0.
            sinT = 1.0 - dTheta * dTheta / 6.0;
            cosT = dTheta / 2.0;
        } else {
            sinT = Math.sin(dTheta) / dTheta;
            cosT = (1.0 - Math.cos(dTheta)) / dTheta;
        }

        Vector2d local = new Vector2d(dx * sinT - dy * cosT, dx * cosT + dy * sinT);
        return new Pose2d(pose.position.plus(local.rotated(pose.heading)),
                Angles.normalize(pose.heading + dTheta));
    }

    public Twist2d times(double s) {
        return new Twist2d(dx * s, dy * s, dTheta * s);
    }

    @Override
    public String toString() {
        return String.format("Twist2d(%.4f, %.4f, %.4f)", dx, dy, dTheta);
    }
}
