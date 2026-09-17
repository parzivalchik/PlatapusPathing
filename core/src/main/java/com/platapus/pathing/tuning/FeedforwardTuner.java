package com.platapus.pathing.tuning;

import com.platapus.pathing.math.Matrix3;

/**
 * Fits {@code kS}, {@code kV} and {@code kA} online by recursive least squares.
 *
 * <p>The feedforward model is linear in its parameters:
 *
 * <pre>{@code
 * u = kS * sign(v) + kV * v + kA * a
 *   = theta . x,   theta = [kS, kV, kA],  x = [sign(v), v, a]
 * }</pre>
 *
 * <p>so every control loop hands this class one regressor row and one observed
 * normalised voltage, and least squares does the rest. No dedicated tuning
 * OpMode, no driving the robot back and forth at fixed powers while reading a
 * graph -- the model fits itself from whatever the robot happened to do.
 *
 * <p>{@code u} must be the <b>battery-normalised</b> applied voltage,
 * {@code duty * vBattery / 12}, not the raw duty cycle. See {@link
 * FeedforwardGains}.
 *
 * <h2>Forgetting</h2>
 * Old samples decay by {@code forgettingFactor} each step, so the fit tracks a
 * robot that changes during a match -- a belt tightening up, a wheel picking up
 * grit, a battery aging. Values near 1 average over a long history and are
 * stable but slow; lower values chase recent data and get twitchy. 0.98 to 0.995
 * is the usable band, which is roughly a 200 to 2000 sample memory.
 *
 * <h2>What this class deliberately does not do</h2>
 * It never touches a running follower. It produces an estimate; deciding when
 * that estimate is safe to adopt is {@link TunableFeedforward}'s job, and the
 * answer is never "in the middle of a path".
 */
public final class FeedforwardTuner {

    /** Below this many samples, confidence is reported as zero regardless. */
    private static final int MIN_SAMPLES_FOR_CONFIDENCE = 20;

    private final double forgettingFactor;
    private final double minVelocity;
    private final double maxCommand;

    private double[] theta;
    private Matrix3 covariance;
    /** Forgetting-weighted accumulators for x_i^2, and their total weight. */
    private final double[] regressorEnergy = new double[3];
    private double residualEnergy;
    private double weightSum;

    private long sampleCount;
    private long rejectedCount;
    private double sumSquaredResidual;

    private FeedforwardTuner(Builder b) {
        this.forgettingFactor = b.forgettingFactor;
        this.minVelocity = b.minVelocity;
        this.maxCommand = b.maxCommand;
        this.theta = new double[]{b.initial.kS, b.initial.kV, b.initial.kA};
        double p = b.initialCovariance;
        this.covariance = Matrix3.diagonal(p, p, p);
    }

    public static Builder builder(FeedforwardGains initial) {
        return new Builder(initial);
    }

    public static final class Builder {
        private final FeedforwardGains initial;
        private double forgettingFactor = 0.99;
        private double minVelocity = 2.0;
        private double maxCommand = 0.95;
        private double initialCovariance = 1e-3;

        private Builder(FeedforwardGains initial) {
            if (initial == null) {
                throw new IllegalArgumentException("initial gains must be non-null");
            }
            this.initial = initial;
        }

        /** Exponential forgetting factor, in (0, 1]. Sensible range 0.98 to 0.995. */
        public Builder forgettingFactor(double lambda) {
            this.forgettingFactor = lambda;
            return this;
        }

        /**
         * Samples below this wheel speed, inches per second, are discarded.
         *
         * <p>{@code sign(v)} is the kS regressor, and near zero velocity its sign
         * flips on sensor noise. Those rows carry no information about static
         * friction and actively corrupt the fit, so they are dropped rather than
         * down-weighted.
         */
        public Builder minVelocity(double inchesPerSecond) {
            this.minVelocity = inchesPerSecond;
            return this;
        }

        /**
         * Samples at or above this normalised command are discarded.
         *
         * <p>A saturated motor is not obeying the model -- the robot asked for
         * more than full power and got full power. Fitting to those rows teaches
         * the model that large velocities are cheap, which biases kV down and
         * makes every subsequent auto undershoot.
         */
        public Builder maxCommand(double command) {
            this.maxCommand = command;
            return this;
        }

        /**
         * Initial parameter covariance. Larger means the starting gains are held
         * loosely and the fit moves quickly at first.
         */
        public Builder initialCovariance(double p) {
            this.initialCovariance = p;
            return this;
        }

        public FeedforwardTuner build() {
            if (forgettingFactor <= 0 || forgettingFactor > 1.0) {
                throw new IllegalArgumentException(
                        "forgettingFactor must be in (0, 1]; got " + forgettingFactor);
            }
            if (initialCovariance <= 0) {
                throw new IllegalArgumentException("initialCovariance must be positive");
            }
            return new FeedforwardTuner(this);
        }
    }

    /**
     * Offers one observation to the fit.
     *
     * @param normalisedCommand the applied command, {@code duty * vBattery / 12}
     * @param velocity          measured wheel velocity, inches per second
     * @param acceleration      wheel acceleration, inches per second squared
     * @return true if the sample was used, false if it was rejected as
     *         uninformative or saturated
     */
    public boolean addSample(double normalisedCommand, double velocity, double acceleration) {
        if (!isUsable(normalisedCommand, velocity, acceleration)) {
            rejectedCount++;
            return false;
        }

        double[] x = {Math.signum(velocity), velocity, acceleration};

        // Standard RLS with exponential forgetting:
        //   K     = P x / (lambda + x' P x)
        //   theta = theta + K (u - theta' x)
        //   P     = (P - K (P x)') / lambda
        double[] px = covariance.times(x);
        double denominator = forgettingFactor + Matrix3.dot(x, px);
        if (denominator < 1e-12) {
            rejectedCount++;
            return false;
        }

        double residual = normalisedCommand - Matrix3.dot(theta, x);
        double[] gain = {px[0] / denominator, px[1] / denominator, px[2] / denominator};

        theta = new double[]{
                theta[0] + gain[0] * residual,
                theta[1] + gain[1] * residual,
                theta[2] + gain[2] * residual};

        // P is symmetric, so x' P is the same vector as P x.
        covariance = covariance.minus(Matrix3.outer(gain, px))
                .times(1.0 / forgettingFactor)
                .symmetrized();

        // Forgetting-weighted running means, for the confidence report. Kept as
        // energy over weight rather than an exponential average seeded at zero,
        // so the first few samples give an honest mean instead of one biased
        // toward the zero it started at.
        weightSum = forgettingFactor * weightSum + 1.0;
        for (int i = 0; i < 3; i++) {
            regressorEnergy[i] = forgettingFactor * regressorEnergy[i] + x[i] * x[i];
        }
        residualEnergy = forgettingFactor * residualEnergy + residual * residual;

        sampleCount++;
        sumSquaredResidual += residual * residual;
        return true;
    }

    private boolean isUsable(double command, double velocity, double acceleration) {
        return Double.isFinite(command)
                && Double.isFinite(velocity)
                && Double.isFinite(acceleration)
                && Math.abs(velocity) >= minVelocity
                && Math.abs(command) < maxCommand;
    }

    /** The current fit. */
    public FeedforwardGains getGains() {
        return new FeedforwardGains(theta[0], theta[1], theta[2]);
    }

    /** How many samples have been folded into the fit. */
    public long getSampleCount() {
        return sampleCount;
    }

    /** How many samples were offered and thrown away. */
    public long getRejectedCount() {
        return rejectedCount;
    }

    /** Root mean squared residual across every accepted sample. */
    public double getRmsResidual() {
        return sampleCount == 0 ? Double.NaN : Math.sqrt(sumSquaredResidual / sampleCount);
    }

    /**
     * A bounded summary of how well determined the fit is, in [0, 1].
     *
     * <p>Not the raw covariance trace. The three regressors differ by orders of
     * magnitude -- {@code sign(v)} is around 1 while {@code v} is around 50 --
     * so their variances are not remotely comparable, and a trace is dominated by
     * whichever parameter happens to carry the smallest regressor. Worse, under
     * forgetting the covariance settles at a steady state rather than shrinking
     * toward zero, so a raw trace can quite legitimately <i>grow</i> as data
     * arrives.
     *
     * <p>What is comparable, and what actually matters, is how much each
     * parameter's uncertainty contributes to the uncertainty of a
     * <i>prediction</i>: {@code sigma^2 * P[i][i] * E[x_i^2]}, which is in
     * command units squared for every parameter. Summing those gives the variance
     * of the command this model predicts, reported here against a 2% reference.
     *
     * <p>The {@code sigma^2} factor is essential and easy to leave out. Under
     * forgetting, {@code P} settles at a floor proportional to {@code 1 - lambda}
     * rather than decaying toward zero, so {@code P} alone claims a fixed
     * uncertainty no matter how cleanly the data fits. {@code P} is an inverse
     * information matrix, not a covariance; the covariance is the residual
     * variance times {@code P}. Scaling by the measured residual is what makes a
     * model that predicts its data perfectly read as certain.
     *
     * <p>A robot that has only ever driven at one speed, and therefore cannot
     * separate kS from kV, reads low -- which is the point.
     */
    public double getConfidence() {
        if (sampleCount < MIN_SAMPLES_FOR_CONFIDENCE) {
            // Too little data to say anything. Reporting certainty here would be
            // worse than useless, since it gates whether a fit gets adopted.
            return 0.0;
        }
        double variance = predictionVariance();
        if (!Double.isFinite(variance) || variance < 0) {
            return 0.0;
        }
        double reference = 0.02 * 0.02;
        return reference / (reference + variance);
    }

    /**
     * Standard deviation of the command this model predicts, in command units.
     * Roughly "how many percent of full power might this model be wrong by".
     */
    public double getPredictionStdDev() {
        return Math.sqrt(Math.max(0, predictionVariance()));
    }

    private double predictionVariance() {
        if (weightSum < 1e-9) {
            return Double.POSITIVE_INFINITY;
        }
        double residualVariance = residualEnergy / weightSum;
        double sum = 0;
        for (int i = 0; i < 3; i++) {
            sum += Math.max(0, covariance.get(i, i)) * (regressorEnergy[i] / weightSum);
        }
        return residualVariance * sum;
    }

    /** Discards the fit and restarts from {@code gains}. */
    public void reset(FeedforwardGains gains, double initialCovariance) {
        if (gains == null) {
            throw new IllegalArgumentException("gains must be non-null");
        }
        this.theta = new double[]{gains.kS, gains.kV, gains.kA};
        this.covariance = Matrix3.diagonal(
                initialCovariance, initialCovariance, initialCovariance);
        this.sampleCount = 0;
        this.rejectedCount = 0;
        this.sumSquaredResidual = 0;
        this.residualEnergy = 0;
        this.weightSum = 0;
        java.util.Arrays.fill(this.regressorEnergy, 0.0);
    }
}
