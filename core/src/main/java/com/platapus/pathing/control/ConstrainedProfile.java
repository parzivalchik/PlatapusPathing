package com.platapus.pathing.control;

/**
 * A motion profile that respects a velocity ceiling which varies along the path.
 *
 * <p>{@link MotionProfile} assumes one speed limit for the whole distance and
 * solves it in closed form. That is not enough once a segment wants its own cap
 * -- a slow, careful approach into a scoring position followed by a fast run back
 * across the field. A single trapezoid can only be as fast as the slowest part of
 * the path.
 *
 * <p>So this solves it numerically instead, with the standard forward-backward
 * sweep:
 *
 * <ol>
 *   <li>Sample the path at a fixed arc-length step and read the velocity ceiling
 *       at each point.</li>
 *   <li><b>Forward pass</b>, from rest: at each step, go no faster than
 *       acceleration could have got you there from the previous step. This
 *       enforces "you cannot speed up faster than the robot can".</li>
 *   <li><b>Backward pass</b>, from rest at the end: at each step, go no faster
 *       than you could still brake from to meet the next step's speed. This
 *       enforces "you must already be slowing down before you reach the slow
 *       bit" -- which is the part a naive per-segment profile gets wrong, by
 *       arriving at a slow segment still going fast and then braking harder than
 *       the robot can.</li>
 *   <li>Integrate the resulting speeds to get a time for each sample.</li>
 * </ol>
 *
 * <p>With a constant ceiling this reproduces {@link MotionProfile}'s trapezoid,
 * which is asserted in the tests.
 */
public final class ConstrainedProfile {

    /** The velocity ceiling at a point along the path. */
    @FunctionalInterface
    public interface VelocityLimit {
        /** Maximum permitted speed, inches per second, at {@code arcLength} inches. */
        double maxVelocityAt(double arcLength);

        /** The same ceiling everywhere. */
        static VelocityLimit constant(double maxVelocity) {
            return s -> maxVelocity;
        }
    }

    /** Target arc-length step, inches. Fine enough that a 0.5" feature is seen. */
    private static final double TARGET_STEP = 0.25;
    private static final int MIN_SAMPLES = 32;
    private static final int MAX_SAMPLES = 4000;

    private final double distance;
    private final double step;
    private final double[] velocities;
    private final double[] times;
    private final double duration;
    private final double peakVelocity;

    public ConstrainedProfile(double distance, VelocityLimit limit,
                              double maxAcceleration, double maxDeceleration) {
        if (distance < 0) {
            throw new IllegalArgumentException("distance must be non-negative, got " + distance);
        }
        if (limit == null) {
            throw new IllegalArgumentException("limit must be non-null");
        }
        if (maxAcceleration <= 0 || maxDeceleration <= 0) {
            throw new IllegalArgumentException(
                    "acceleration and deceleration must both be positive");
        }

        this.distance = distance;

        if (distance < 1e-9) {
            this.step = 0;
            this.velocities = new double[]{0};
            this.times = new double[]{0};
            this.duration = 0;
            this.peakVelocity = 0;
            return;
        }

        int samples = (int) Math.ceil(distance / TARGET_STEP);
        samples = Math.max(MIN_SAMPLES, Math.min(MAX_SAMPLES, samples));
        this.step = distance / samples;

        double[] v = new double[samples + 1];

        // Ceiling at each sample. A non-positive or non-finite limit would stall
        // the profile forever, so it is floored at something crawling.
        for (int i = 0; i <= samples; i++) {
            double ceiling = limit.maxVelocityAt(i * step);
            v[i] = (Double.isFinite(ceiling) && ceiling > 1e-3) ? ceiling : 1e-3;
        }

        // Forward: cannot accelerate harder than the robot can.
        v[0] = 0;
        for (int i = 1; i <= samples; i++) {
            v[i] = Math.min(v[i], Math.sqrt(v[i - 1] * v[i - 1] + 2 * maxAcceleration * step));
        }

        // Backward: must already be braking before anything slower.
        v[samples] = 0;
        for (int i = samples - 1; i >= 0; i--) {
            v[i] = Math.min(v[i], Math.sqrt(v[i + 1] * v[i + 1] + 2 * maxDeceleration * step));
        }

        this.velocities = v;
        this.times = new double[samples + 1];
        double peak = 0;
        for (int i = 0; i < samples; i++) {
            // Constant acceleration over the step, so the average speed is the
            // mean of the endpoints and dt follows directly.
            double meanSpeed = 0.5 * (v[i] + v[i + 1]);
            times[i + 1] = times[i] + (meanSpeed < 1e-9 ? 0 : step / meanSpeed);
            peak = Math.max(peak, v[i]);
        }
        this.peakVelocity = Math.max(peak, v[samples]);
        this.duration = times[samples];
    }

    /**
     * The fastest a robot may take a corner of the given curvature without
     * exceeding a lateral acceleration limit.
     *
     * <p>On a curve of radius {@code r}, travelling at {@code v} costs
     * {@code v^2 / r} of sideways acceleration, and curvature is {@code 1/r}, so
     * the limit falls straight out:
     *
     * <pre>{@code v <= sqrt(a_lat / |curvature|)}</pre>
     *
     * <p>Without this the profile will happily plan a hairpin at top speed. The
     * robot then either slides -- at which point the wheels are measuring
     * something the chassis is not doing, and odometry goes with it -- or simply
     * fails to turn that tightly and cuts the corner.
     *
     * <p>Returns infinity on a straight line, or when no limit is configured.
     *
     * @param curvature              signed curvature, 1/inches
     * @param maxLateralAcceleration inches per second squared, or infinity
     */
    public static double lateralAccelerationLimit(double curvature,
                                                  double maxLateralAcceleration) {
        double magnitude = Math.abs(curvature);
        if (magnitude < 1e-9
                || !Double.isFinite(maxLateralAcceleration)
                || maxLateralAcceleration <= 0) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.sqrt(maxLateralAcceleration / magnitude);
    }

    /** A profile with one speed limit throughout. Equivalent to a trapezoid. */
    public ConstrainedProfile(double distance, double maxVelocity,
                              double maxAcceleration, double maxDeceleration) {
        this(distance, VelocityLimit.constant(maxVelocity), maxAcceleration, maxDeceleration);
    }

    /** The profile state at {@code time} seconds after the start. */
    public MotionState get(double time) {
        if (distance < 1e-9) {
            return new MotionState(0, 0, 0);
        }
        if (time <= 0) {
            return new MotionState(0, 0, accelerationAt(0));
        }
        if (time >= duration) {
            return new MotionState(distance, 0, 0);
        }

        int i = indexBefore(time);
        double span = times[i + 1] - times[i];
        double fraction = span < 1e-12 ? 0 : (time - times[i]) / span;

        double v = velocities[i] + (velocities[i + 1] - velocities[i]) * fraction;
        // Position within the step, integrating the linear speed ramp.
        double position = i * step + 0.5 * (velocities[i] + v) * (time - times[i]);

        return new MotionState(Math.min(position, distance), Math.max(v, 0),
                accelerationAt(i));
    }

    private double accelerationAt(int i) {
        if (i >= velocities.length - 1 || step < 1e-12) {
            return 0;
        }
        double v0 = velocities[i];
        double v1 = velocities[i + 1];
        return (v1 * v1 - v0 * v0) / (2 * step);
    }

    private int indexBefore(double time) {
        int low = 0;
        int high = times.length - 1;
        while (high - low > 1) {
            int mid = (low + high) >>> 1;
            if (times[mid] <= time) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return low;
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

    /** The planned speed at {@code arcLength} inches along the path. */
    public double velocityAtArcLength(double arcLength) {
        if (distance < 1e-9) {
            return 0;
        }
        double clamped = Math.max(0, Math.min(distance, arcLength));
        double scaled = clamped / step;
        int i = (int) Math.floor(scaled);
        if (i >= velocities.length - 1) {
            return velocities[velocities.length - 1];
        }
        double fraction = scaled - i;
        return velocities[i] + (velocities[i + 1] - velocities[i]) * fraction;
    }

    @Override
    public String toString() {
        return String.format("ConstrainedProfile(%.2f\" in %.2fs, peak %.1f\"/s)",
                distance, duration, peakVelocity);
    }
}
