package com.platapus.pathing.tuning;

import com.platapus.pathing.control.Clock;
import com.platapus.pathing.drive.Drivetrain;

/**
 * Runs the feedforward fit from live robot data.
 *
 * <p>Call {@link #update(double[])} once per control loop with the powers just
 * commanded. It reads back what the wheels actually did, converts the commanded
 * duty into a battery-normalised voltage, differentiates velocity to get
 * acceleration, and feeds one row per wheel into the {@link FeedforwardTuner}.
 *
 * <p>All four wheels feed a single model, because the follower applies a single
 * set of constants to all four. Fitting per-wheel models would describe the robot
 * more precisely and then have nowhere to put the answer.
 *
 * <p>Nothing here ever changes what the follower is doing. It proposes; the
 * follower adopts at its next path boundary. See {@link TunableFeedforward}.
 */
public final class TuningSession {

    /** Nominal battery voltage the normalised command is expressed against. */
    public static final double NOMINAL_VOLTAGE = 12.0;

    private final Drivetrain drivetrain;
    private final VoltageSource voltageSource;
    private final Clock clock;
    private final FeedforwardTuner tuner;
    private final TunableFeedforward feedforward;
    private final TuningStore store;
    private final long minSamplesBeforeProposing;
    private final double minConfidenceBeforeProposing;

    private double[] lastVelocities;
    private double lastTime = Double.NaN;
    private boolean proposedThisSession;

    private TuningSession(Builder b) {
        this.drivetrain = b.drivetrain;
        this.voltageSource = b.voltageSource;
        this.clock = b.clock;
        this.feedforward = b.feedforward;
        this.store = b.store;
        this.minSamplesBeforeProposing = b.minSamplesBeforeProposing;
        this.minConfidenceBeforeProposing = b.minConfidenceBeforeProposing;
        this.tuner = FeedforwardTuner.builder(b.feedforward.get())
                .forgettingFactor(b.forgettingFactor)
                .build();
    }

    public static Builder builder(Drivetrain drivetrain, VoltageSource voltageSource,
                                  Clock clock, TunableFeedforward feedforward) {
        return new Builder(drivetrain, voltageSource, clock, feedforward);
    }

    public static final class Builder {
        private final Drivetrain drivetrain;
        private final VoltageSource voltageSource;
        private final Clock clock;
        private final TunableFeedforward feedforward;
        private TuningStore store;
        private double forgettingFactor = 0.99;
        private long minSamplesBeforeProposing = 400;
        private double minConfidenceBeforeProposing = 0.5;

        private Builder(Drivetrain drivetrain, VoltageSource voltageSource,
                        Clock clock, TunableFeedforward feedforward) {
            if (drivetrain == null || voltageSource == null || clock == null
                    || feedforward == null) {
                throw new IllegalArgumentException("all arguments must be non-null");
            }
            this.drivetrain = drivetrain;
            this.voltageSource = voltageSource;
            this.clock = clock;
            this.feedforward = feedforward;
        }

        /** See {@link FeedforwardTuner.Builder#forgettingFactor(double)}. */
        public Builder forgettingFactor(double lambda) {
            this.forgettingFactor = lambda;
            return this;
        }

        /**
         * How many accepted samples before a fit is offered for adoption. At
         * four wheels and 50 Hz that is two seconds of driving per 400 samples.
         */
        public Builder minSamplesBeforeProposing(long samples) {
            this.minSamplesBeforeProposing = samples;
            return this;
        }

        /** How well determined the fit must be before it is offered. */
        public Builder minConfidenceBeforeProposing(double confidence) {
            this.minConfidenceBeforeProposing = confidence;
            return this;
        }

        /** Where to persist an adopted fit. Optional. */
        public Builder store(TuningStore store) {
            this.store = store;
            return this;
        }

        public TuningSession build() {
            return new TuningSession(this);
        }
    }

    /**
     * Folds one loop's worth of data into the fit.
     *
     * @param commandedPowers the powers just written to the drivetrain, [-1, 1]
     * @return how many wheel samples were accepted this loop
     */
    public int update(double[] commandedPowers) {
        if (commandedPowers == null) {
            return 0;
        }

        double now = clock.seconds();
        double[] velocities;
        try {
            velocities = drivetrain.getWheelVelocities();
        } catch (UnsupportedOperationException e) {
            // A drivetrain that cannot report velocity cannot be tuned this way.
            // Not an error worth aborting an auto over; the fit simply does not
            // advance and the compiled-in constants stay in force.
            return 0;
        }

        if (lastVelocities == null || Double.isNaN(lastTime)) {
            lastVelocities = velocities.clone();
            lastTime = now;
            return 0;
        }

        double dt = now - lastTime;
        lastTime = now;
        if (dt <= 1e-6) {
            return 0;
        }

        double voltage = voltageSource.getVoltage();
        // A nonsense battery reading would poison every sample this loop.
        boolean voltageUsable = Double.isFinite(voltage) && voltage > 6.0 && voltage < 20.0;

        int accepted = 0;
        int count = Math.min(commandedPowers.length, velocities.length);
        for (int i = 0; i < count && voltageUsable; i++) {
            double acceleration = (velocities[i] - lastVelocities[i]) / dt;
            double normalised = commandedPowers[i] * voltage / NOMINAL_VOLTAGE;
            if (tuner.addSample(normalised, velocities[i], acceleration)) {
                accepted++;
            }
        }
        lastVelocities = velocities.clone();

        maybePropose();
        return accepted;
    }

    private void maybePropose() {
        if (tuner.getSampleCount() < minSamplesBeforeProposing
                || tuner.getConfidence() < minConfidenceBeforeProposing) {
            return;
        }
        if (feedforward.propose(tuner.getGains())) {
            proposedThisSession = true;
        }
    }

    /**
     * Writes the currently active gains to the store, if there is one.
     *
     * <p>Call at the end of an OpMode. Saves what the follower actually used --
     * the active gains, not the raw fit -- so a restarted OpMode picks up
     * exactly where this one left off rather than adopting a fit that was never
     * screened.
     */
    public boolean persist() {
        return store != null && store.save(feedforward.get());
    }

    /**
     * Loads saved gains into the feedforward, replacing the compiled-in
     * defaults. Call once at OpMode init, before anything starts moving.
     *
     * @return true if something was loaded
     */
    public boolean restore() {
        if (store == null) {
            return false;
        }
        FeedforwardGains saved = store.load();
        if (saved == null) {
            return false;
        }
        feedforward.setActiveImmediately(saved);
        tuner.reset(saved, 1e-3);
        return true;
    }

    public FeedforwardTuner getTuner() {
        return tuner;
    }

    /** The fit as it currently stands, which may not be what the follower uses. */
    public FeedforwardGains getFittedGains() {
        return tuner.getGains();
    }

    /** Whether a fit has ever been good enough to offer for adoption. */
    public boolean hasProposed() {
        return proposedThisSession;
    }
}
