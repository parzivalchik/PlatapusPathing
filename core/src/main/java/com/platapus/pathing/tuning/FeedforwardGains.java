package com.platapus.pathing.tuning;

/**
 * The three feedforward constants, as one immutable value.
 *
 * <p>Motor command for a wheel is {@code kS * sign(v) + kV * v + kA * a}, where
 * {@code v} is inches per second and {@code a} is inches per second squared. The
 * result is a <b>battery-normalised</b> command in [-1, 1]: it is the fraction of
 * <i>nominal</i> (12 V) voltage the wheel needs, not a raw duty cycle. Converting
 * that to a duty cycle at the current battery voltage is the drivetrain's job.
 *
 * <p>That distinction matters more than it looks. A duty cycle of 0.5 means
 * something different at 13 V than at 11 V, so a model fitted against raw duty
 * silently re-fits itself every time the battery sags -- which is exactly when
 * autos start missing.
 */
public final class FeedforwardGains {

    public final double kS;
    public final double kV;
    public final double kA;

    public FeedforwardGains(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /** The normalised command this model predicts for a velocity and acceleration. */
    public double predict(double velocity, double acceleration) {
        double staticTerm = Math.abs(velocity) > 1e-3 ? Math.signum(velocity) * kS : 0.0;
        return staticTerm + kV * velocity + kA * acceleration;
    }

    /**
     * These gains with tiny negative {@code kS} or {@code kA} clamped to zero.
     *
     * <p>Least squares has no idea that static friction cannot be negative. On a
     * robot with very little of it the fit lands somewhere around zero and lands
     * slightly under about half the time, purely from noise. Rejecting the whole
     * fit over a {@code kS} of -0.0004 would throw away a perfectly good {@code
     * kV}; clamping keeps the useful part and discards the meaningless sign.
     *
     * <p>A <i>large</i> negative value is not noise -- it means the fit has gone
     * wrong -- so it survives clamping and is caught by {@link #isPlausible()}.
     */
    public FeedforwardGains sanitized() {
        double clampedS = (kS < 0 && kS > -0.02) ? 0.0 : kS;
        double clampedA = (kA < 0 && kA > -0.02) ? 0.0 : kA;
        return (clampedS == kS && clampedA == kA)
                ? this : new FeedforwardGains(clampedS, kV, clampedA);
    }

    /** Whether every constant is finite and physically sensible. */
    public boolean isPlausible() {
        return Double.isFinite(kS) && Double.isFinite(kV) && Double.isFinite(kA)
                && kV > 0 && kS >= 0 && kA >= 0
                && kS < 1.0 && kV < 1.0 && kA < 1.0;
    }

    /**
     * How far these gains sit from {@code other}, as a fraction of {@code
     * other}'s magnitude. Used to decide whether a newly fitted model has moved
     * enough to be worth adopting.
     */
    public double relativeDistanceFrom(FeedforwardGains other) {
        double scale = Math.abs(other.kS) + Math.abs(other.kV) + Math.abs(other.kA);
        if (scale < 1e-12) {
            return Double.POSITIVE_INFINITY;
        }
        return (Math.abs(kS - other.kS) + Math.abs(kV - other.kV) + Math.abs(kA - other.kA))
                / scale;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof FeedforwardGains)) {
            return false;
        }
        FeedforwardGains g = (FeedforwardGains) o;
        return Double.compare(kS, g.kS) == 0
                && Double.compare(kV, g.kV) == 0
                && Double.compare(kA, g.kA) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(kS) * 31 * 31 + Double.hashCode(kV) * 31 + Double.hashCode(kA);
    }

    @Override
    public String toString() {
        return String.format("FeedforwardGains(kS=%.5f, kV=%.5f, kA=%.5f)", kS, kV, kA);
    }
}
