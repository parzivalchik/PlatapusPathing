package com.platapus.pathing.tuning;

import java.util.ArrayList;
import java.util.List;

/**
 * Watches each drive motor's current draw against what the tuned feedforward
 * model says it ought to be, and reports the ones that disagree for long enough
 * to matter.
 *
 * <h2>Why not just a current threshold</h2>
 * A fixed "over 6 amps means stalled" trips constantly during hard acceleration,
 * when a healthy motor legitimately pulls a lot, and misses a genuine stall at
 * low commanded power, where a wedged wheel might only draw three. Current alone
 * does not say whether a motor is in trouble; current <i>compared with what this
 * motion should cost</i> does.
 *
 * <h2>The model</h2>
 * A DC motor's electrical behaviour is
 *
 * <pre>{@code V_applied = I * R + V_backEmf}</pre>
 *
 * so the current follows from the applied voltage and the speed:
 *
 * <pre>{@code I = (V_applied - V_backEmf) / R}</pre>
 *
 * <p>Both terms come from things already known. {@code V_applied} is the
 * commanded duty times the battery voltage. The back EMF is exactly what the
 * feedforward's {@code kV * v} term represents -- the fraction of nominal
 * voltage needed to sustain that speed -- so it is {@code kV * v * 12}. And
 * {@code R} follows from the motor's stall current on its datasheet, since a
 * stalled motor has no back EMF at all: {@code R = 12 / I_stall}.
 *
 * <h2>Commanded speed, not measured speed</h2>
 * The back-EMF term uses the velocity the follower <b>asked</b> for, not the one
 * the encoder reports. This is the entire trick, and getting it backwards makes
 * the detector useless.
 *
 * <p>Predicting from measured velocity is self-fulfilling: a wedged wheel reads
 * zero speed, so the model predicts zero back EMF and therefore a large current
 * -- exactly the large current the motor is really drawing. Measured and
 * predicted agree perfectly and nothing ever looks wrong.
 *
 * <p>Predicting from commanded velocity asks a different and useful question:
 * "if this wheel were turning as fast as we told it to, what would it cost?" A
 * healthy wheel is spinning, generating back EMF, and drawing little. A stalled
 * one generates none, so it draws several times the prediction. That gap is the
 * signal.
 *
 * <h2>Debouncing</h2>
 * Current readings on a Control Hub are noisy, and a momentary spike crossing a
 * wheel seam is not a stall. The condition must hold continuously for {@code
 * debounceSeconds} -- 150 to 300 ms is the useful range -- before anything fires.
 */
public final class StallDetector {

    private final TunableFeedforward feedforward;
    private final VoltageSource voltageSource;
    private final double resistanceOhms;
    private final double currentRatioThreshold;
    private final double minAbsoluteExcessAmps;
    private final double debounceSeconds;
    private final List<StallListener> listeners = new ArrayList<>();

    /** When each motor's over-current condition began, or NaN if it is not. */
    private double[] conditionStart;
    private boolean[] reported;

    private StallDetector(Builder b) {
        this.feedforward = b.feedforward;
        this.voltageSource = b.voltageSource;
        this.resistanceOhms = TuningSession.NOMINAL_VOLTAGE / b.stallCurrentAmps;
        this.currentRatioThreshold = b.currentRatioThreshold;
        this.minAbsoluteExcessAmps = b.minAbsoluteExcessAmps;
        this.debounceSeconds = b.debounceSeconds;
    }

    public static Builder builder(TunableFeedforward feedforward, VoltageSource voltageSource) {
        return new Builder(feedforward, voltageSource);
    }

    public static final class Builder {
        private final TunableFeedforward feedforward;
        private final VoltageSource voltageSource;
        private double stallCurrentAmps = 9.2; // goBILDA 5202/5203 series
        private double currentRatioThreshold = 2.0;
        private double minAbsoluteExcessAmps = 1.5;
        private double debounceSeconds = 0.2;

        private Builder(TunableFeedforward feedforward, VoltageSource voltageSource) {
            if (feedforward == null || voltageSource == null) {
                throw new IllegalArgumentException(
                        "feedforward and voltageSource must be non-null");
            }
            this.feedforward = feedforward;
            this.voltageSource = voltageSource;
        }

        /**
         * The motor's stall current at 12 V, amps, from its datasheet. Winding
         * resistance is derived from it. 9.2 A for a goBILDA Yellow Jacket.
         */
        public Builder stallCurrentAmps(double amps) {
            this.stallCurrentAmps = amps;
            return this;
        }

        /**
         * How many times the predicted current counts as a stall. 2.0 means
         * "drawing twice what this motion should cost".
         */
        public Builder currentRatioThreshold(double ratio) {
            this.currentRatioThreshold = ratio;
            return this;
        }

        /**
         * How many amps above prediction are also required, regardless of ratio.
         *
         * <p>Guards the low end: when the model predicts 0.2 A, a perfectly
         * healthy 0.5 A reading is two and a half times prediction and means
         * nothing. Requiring an absolute excess as well stops a coasting robot
         * reporting four stalls a second.
         */
        public Builder minAbsoluteExcessAmps(double amps) {
            this.minAbsoluteExcessAmps = amps;
            return this;
        }

        /** How long the condition must hold before firing. 0.15 to 0.3 s. */
        public Builder debounceSeconds(double seconds) {
            this.debounceSeconds = seconds;
            return this;
        }

        public StallDetector build() {
            if (stallCurrentAmps <= 0) {
                throw new IllegalArgumentException("stallCurrentAmps must be positive");
            }
            if (currentRatioThreshold <= 1.0) {
                throw new IllegalArgumentException(
                        "currentRatioThreshold must exceed 1; got " + currentRatioThreshold);
            }
            if (debounceSeconds < 0) {
                throw new IllegalArgumentException("debounceSeconds must be non-negative");
            }
            return new StallDetector(this);
        }
    }

    public void addListener(StallListener listener) {
        if (listener == null) {
            throw new IllegalArgumentException("listener must be non-null");
        }
        listeners.add(listener);
    }

    public void removeListener(StallListener listener) {
        listeners.remove(listener);
    }

    /**
     * The current this model expects a wheel to draw.
     *
     * @param commandedPower    the duty cycle written to the motor, [-1, 1]
     * @param commandedVelocity the wheel velocity the follower asked for, inches
     *                          per second -- <b>not</b> the measured one
     * @param batteryVoltage    present battery voltage, volts
     * @return expected current magnitude, amps
     */
    public double expectedCurrent(double commandedPower, double commandedVelocity,
                                  double batteryVoltage) {
        double applied = commandedPower * batteryVoltage;
        double backEmf =
                feedforward.get().kV * commandedVelocity * TuningSession.NOMINAL_VOLTAGE;
        return Math.abs(applied - backEmf) / resistanceOhms;
    }

    /**
     * Checks every motor once. Call each control loop, after commanding powers.
     *
     * @param timeSeconds         clock time now
     * @param commandedPowers     what was just written to each motor, [-1, 1]
     * @param commandedVelocities the wheel velocities the follower asked for,
     *                            inches per second. See the note above on why
     *                            these must be commanded, not measured.
     * @param currents            measured currents, amps
     */
    public void update(double timeSeconds, double[] commandedPowers,
                       double[] commandedVelocities, double[] currents) {
        if (commandedPowers == null || commandedVelocities == null || currents == null) {
            return;
        }

        int count = Math.min(commandedPowers.length,
                Math.min(commandedVelocities.length, currents.length));
        ensureCapacity(count);

        double voltage = voltageSource.getVoltage();
        if (!Double.isFinite(voltage) || voltage <= 6.0) {
            return;
        }

        for (int i = 0; i < count; i++) {
            double expected =
                    expectedCurrent(commandedPowers[i], commandedVelocities[i], voltage);
            double measured = Math.abs(currents[i]);

            boolean overDrawing = measured > expected * currentRatioThreshold
                    && measured - expected > minAbsoluteExcessAmps;

            if (!overDrawing) {
                if (reported[i]) {
                    for (StallListener l : listeners) {
                        l.onStallCleared(i, timeSeconds);
                    }
                }
                conditionStart[i] = Double.NaN;
                reported[i] = false;
                continue;
            }

            if (Double.isNaN(conditionStart[i])) {
                conditionStart[i] = timeSeconds;
                continue;
            }

            double heldFor = timeSeconds - conditionStart[i];
            if (heldFor >= debounceSeconds && !reported[i]) {
                reported[i] = true;
                StallEvent event = new StallEvent(
                        i, measured, expected, commandedVelocities[i], heldFor, timeSeconds);
                for (StallListener l : listeners) {
                    l.onStallDetected(event);
                }
            }
        }
    }

    private void ensureCapacity(int count) {
        if (conditionStart == null || conditionStart.length < count) {
            double[] starts = new double[count];
            boolean[] flags = new boolean[count];
            java.util.Arrays.fill(starts, Double.NaN);
            if (conditionStart != null) {
                System.arraycopy(conditionStart, 0, starts, 0, conditionStart.length);
                System.arraycopy(reported, 0, flags, 0, reported.length);
            }
            conditionStart = starts;
            reported = flags;
        }
    }

    /** Whether motor {@code index} is currently reported as stalled. */
    public boolean isStalled(int index) {
        return reported != null && index < reported.length && reported[index];
    }

    /** Whether any motor is currently reported as stalled. */
    public boolean isAnyStalled() {
        if (reported == null) {
            return false;
        }
        for (boolean r : reported) {
            if (r) {
                return true;
            }
        }
        return false;
    }

    /** Derived winding resistance, ohms. */
    public double getResistanceOhms() {
        return resistanceOhms;
    }

    /** Forgets all debounce state, as if nothing had ever been over-drawing. */
    public void reset() {
        if (conditionStart != null) {
            java.util.Arrays.fill(conditionStart, Double.NaN);
            java.util.Arrays.fill(reported, false);
        }
    }
}
