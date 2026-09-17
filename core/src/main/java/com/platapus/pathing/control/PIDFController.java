package com.platapus.pathing.control;

/**
 * A PID controller with integral clamping and a low-pass filtered derivative.
 *
 * <p>Two details matter on an FTC control loop and are handled here:
 *
 * <ul>
 *   <li><b>Integral windup.</b> While a robot is pinned by a wall or a partner
 *       robot the error never shrinks, and an unclamped integral term keeps
 *       growing until the robot lunges the moment it comes free. The accumulator
 *       is bounded by {@code integralLimit}.
 *   <li><b>Derivative noise.</b> Encoder-derived error is quantised, so raw
 *       {@code de/dt} at a 50 Hz loop is mostly noise, and kD amplifies it into
 *       audible motor chatter. The derivative is passed through a first-order
 *       filter set by {@code derivativeFilter}.
 * </ul>
 */
public final class PIDFController {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double integralLimit;
    private final double derivativeFilter;

    private double integral;
    private double lastError;
    private double filteredDerivative;
    private boolean hasPrevious;

    /**
     * @param integralLimit    maximum absolute value of the integral term's
     *                         contribution to the output. Zero disables I.
     * @param derivativeFilter smoothing in [0, 1). 0 is the raw derivative;
     *                         0.8 is heavy smoothing. Around 0.6 is a sane start.
     */
    public PIDFController(double kP, double kI, double kD, double integralLimit,
                          double derivativeFilter) {
        if (derivativeFilter < 0 || derivativeFilter >= 1.0) {
            throw new IllegalArgumentException(
                    "derivativeFilter must be in [0, 1); got " + derivativeFilter);
        }
        if (integralLimit < 0) {
            throw new IllegalArgumentException("integralLimit must be non-negative");
        }
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralLimit = integralLimit;
        this.derivativeFilter = derivativeFilter;
    }

    /** A plain PD controller: no integral term, lightly filtered derivative. */
    public static PIDFController pd(double kP, double kD) {
        return new PIDFController(kP, 0.0, kD, 0.0, 0.6);
    }

    /**
     * Runs one iteration.
     *
     * @param error       setpoint minus measurement
     * @param dtSeconds   time since the previous call; values of zero or less
     *                    are ignored and only the proportional term is returned
     * @return the controller output, unclamped
     */
    public double calculate(double error, double dtSeconds) {
        if (dtSeconds <= 0) {
            return kP * error;
        }

        if (kI != 0.0) {
            integral += error * dtSeconds;
            double maxAccumulator = integralLimit / Math.abs(kI);
            integral = clamp(integral, -maxAccumulator, maxAccumulator);
        }

        double derivative = 0.0;
        if (hasPrevious) {
            double raw = (error - lastError) / dtSeconds;
            filteredDerivative = derivativeFilter * filteredDerivative
                    + (1.0 - derivativeFilter) * raw;
            derivative = filteredDerivative;
        }
        lastError = error;
        hasPrevious = true;

        return kP * error + kI * integral + kD * derivative;
    }

    /**
     * Clears accumulated state. Call this whenever the controller's job changes
     * -- a new path, a resumed follow -- so that stale integral and derivative
     * from the previous motion do not kick the output on the first loop.
     */
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        filteredDerivative = 0.0;
        hasPrevious = false;
    }

    public double getIntegral() {
        return integral;
    }

    private static double clamp(double v, double min, double max) {
        return v < min ? min : (v > max ? max : v);
    }
}
