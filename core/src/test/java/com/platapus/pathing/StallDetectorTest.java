package com.platapus.pathing;

import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.StallDetector;
import com.platapus.pathing.tuning.StallEvent;
import com.platapus.pathing.tuning.StallListener;
import com.platapus.pathing.tuning.TunableFeedforward;
import com.platapus.pathing.tuning.VoltageSource;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class StallDetectorTest {

    private static final double KV = 0.0165;
    private static final double BATTERY = 12.6;
    private static final double STALL_CURRENT = 9.2;   // goBILDA Yellow Jacket
    private static final double DEBOUNCE = 0.2;

    private TunableFeedforward feedforward;
    private StallDetector detector;
    private List<StallEvent> stalls;
    private List<Integer> cleared;

    @BeforeEach
    void setUp() {
        feedforward = new TunableFeedforward(new FeedforwardGains(0.08, KV, 0.0012));
        detector = StallDetector.builder(feedforward, VoltageSource.constant(BATTERY))
                .stallCurrentAmps(STALL_CURRENT)
                .currentRatioThreshold(2.0)
                .minAbsoluteExcessAmps(1.5)
                .debounceSeconds(DEBOUNCE)
                .build();

        stalls = new ArrayList<>();
        cleared = new ArrayList<>();
        detector.addListener(new StallListener() {
            @Override public void onStallDetected(StallEvent event) { stalls.add(event); }
            @Override public void onStallCleared(int motor, double t) { cleared.add(motor); }
        });
    }

    /**
     * Steps the detector for {@code seconds} with the same state on all four
     * motors. {@code commandedVelocity} is what the follower asked for.
     */
    private double run(double startTime, double seconds, double power,
                       double commandedVelocity, double current) {
        double t = startTime;
        for (; t < startTime + seconds; t += 0.02) {
            detector.update(t,
                    new double[]{power, power, power, power},
                    new double[]{commandedVelocity, commandedVelocity,
                            commandedVelocity, commandedVelocity},
                    new double[]{current, current, current, current});
        }
        return t;
    }

    /** Steps the detector for {@code seconds} with a single motor, for clarity. */
    private double runOneMotor(double startTime, double seconds, double power,
                               double commandedVelocity, double current) {
        double t = startTime;
        for (; t < startTime + seconds; t += 0.02) {
            detector.update(t, new double[]{power},
                    new double[]{commandedVelocity}, new double[]{current});
        }
        return t;
    }

    /** The duty the follower would write to hold {@code velocity} at this battery. */
    private static double dutyFor(double velocity) {
        return feedforwardCommand(velocity) * 12.0 / BATTERY;
    }

    private static double feedforwardCommand(double velocity) {
        return 0.08 * Math.signum(velocity) + KV * velocity;
    }

    @Test
    void resistanceIsDerivedFromTheStallCurrent() {
        // A stalled motor has no back EMF, so R = V_nominal / I_stall.
        assertEquals(12.0 / STALL_CURRENT, detector.getResistanceOhms(), 1e-9);
    }

    @Test
    void expectedCurrentIsZeroWhenTheMotorIsFreewheelingAtItsCommandedSpeed() {
        // Applied voltage exactly balances back EMF: no net drive, no current.
        double velocity = 30.0;
        double duty = KV * velocity * 12.0 / BATTERY;

        assertEquals(0.0, detector.expectedCurrent(duty, velocity, BATTERY), 1e-9);
    }

    @Test
    void healthyDrivingIsPredictedToCostVeryLittleCurrent() {
        // The whole model in one assertion: driving at the speed you asked for
        // costs only the friction term, because back EMF pays for the rest.
        double velocity = 30.0;
        double expected = detector.expectedCurrent(dutyFor(velocity), velocity, BATTERY);

        assertTrue(expected < 1.0,
                "cruising should be cheap, predicted " + expected + " A");
    }

    @Test
    void aStalledWheelIsPredictedToCostFarMoreThanItsCommandedMotionWould() {
        // Same duty, but the wheel is not producing the back EMF that duty
        // assumed. This gap is what the detector actually looks for.
        double velocity = 30.0;
        double duty = dutyFor(velocity);

        double healthy = detector.expectedCurrent(duty, velocity, BATTERY);
        double actualIfStalled = duty * BATTERY / (12.0 / STALL_CURRENT);

        assertTrue(actualIfStalled > healthy * 5,
                "a stall should stand out sharply: " + actualIfStalled
                        + " A vs a predicted " + healthy + " A");
    }

    @Test
    void everyStalledMotorIsReportedSeparately() {
        // All four wedged at once: four events, not one for the group.
        double velocity = 30.0;
        double duty = dutyFor(velocity);
        run(0, DEBOUNCE + 0.1, duty, velocity, duty * BATTERY / (12.0 / STALL_CURRENT));

        assertEquals(4, stalls.size());
        for (int i = 0; i < 4; i++) {
            assertTrue(detector.isStalled(i), "motor " + i);
        }
    }

    @Test
    void aHealthyMotorNeverFires() {
        // Driving normally: current matches prediction closely.
        double velocity = 25.0;
        double power = dutyFor(velocity);
        double expected = detector.expectedCurrent(power, velocity, BATTERY);

        run(0, 3.0, power, velocity, expected * 1.1);

        assertTrue(stalls.isEmpty(), "healthy driving should not report a stall");
        assertFalse(detector.isAnyStalled());
    }

    @Test
    void hardAccelerationDoesNotFireDespiteHighCurrent() {
        // The classic false positive for a fixed threshold: accelerating hard
        // from rest legitimately pulls near stall current, and a plain
        // "over 6 amps" rule would fire every single auto.
        double power = 1.0;
        double commandedVelocity = 3.0;
        double expected = detector.expectedCurrent(power, commandedVelocity, BATTERY);
        assertTrue(expected > 8.0, "this motion genuinely costs a lot: " + expected);

        run(0, 1.0, power, commandedVelocity, expected);

        assertTrue(stalls.isEmpty(),
                "a motor drawing exactly what this motion costs is not stalled");
    }

    @Test
    void aWedgedWheelFiresOnceDebounced() {
        // Commanded to cruise at 30"/s, but jammed: no back EMF, so it draws
        // the full applied voltage across the winding.
        double velocity = 30.0;
        double duty = dutyFor(velocity);
        double stalledCurrent = duty * BATTERY / (12.0 / STALL_CURRENT);

        double t = runOneMotor(0, DEBOUNCE + 0.1, duty, velocity, stalledCurrent);

        assertEquals(1, stalls.size(), "should fire exactly once");
        StallEvent event = stalls.get(0);
        assertEquals(0, event.motorIndex);
        assertEquals(stalledCurrent, event.measuredCurrent, 1e-9);
        assertTrue(event.getCurrentRatio() > 2.0,
                "ratio was " + event.getCurrentRatio());
        assertTrue(detector.isStalled(0));
        assertTrue(t > DEBOUNCE);
    }

    @Test
    void predictingFromMeasuredVelocityWouldMakeTheDetectorBlind() {
        // Regression guard for the subtlest way to break this class. A wedged
        // wheel reads zero speed; if the prediction used that measured zero it
        // would predict the full stall current -- exactly what the motor really
        // draws -- and nothing would ever look anomalous.
        double commandedVelocity = 30.0;
        double duty = dutyFor(commandedVelocity);
        double stalledCurrent = duty * BATTERY / (12.0 / STALL_CURRENT);

        double fromCommanded = detector.expectedCurrent(duty, commandedVelocity, BATTERY);
        double fromMeasured = detector.expectedCurrent(duty, 0.0, BATTERY);

        assertEquals(stalledCurrent, fromMeasured, 1e-9,
                "predicting from the measured zero just reproduces the stall current");
        assertTrue(stalledCurrent / fromMeasured < 1.01,
                "which would give a ratio of 1 and never fire");
        assertTrue(stalledCurrent / fromCommanded > 5.0,
                "while predicting from the commanded speed exposes the stall");
    }

    @Test
    void aBriefSpikeShorterThanTheDebounceIsIgnored() {
        // Crossing a seam or a field element: real, but not a stall.
        run(0, 0.10, dutyFor(30), 30.0, 9.0);
        run(0.10, 0.5, dutyFor(30), 30.0,
                detector.expectedCurrent(dutyFor(30), 30.0, BATTERY));

        assertTrue(stalls.isEmpty(),
                "a 100 ms spike under a 200 ms debounce must not fire");
    }

    @Test
    void theConditionMustHoldContinuouslyNotCumulatively() {
        // Alternating spikes that add up past the debounce but never persist.
        double t = 0;
        for (int i = 0; i < 20; i++) {
            t = run(t, 0.10, dutyFor(30), 30.0, 9.0);
            t = run(t, 0.06, dutyFor(30), 30.0,
                    detector.expectedCurrent(dutyFor(30), 30.0, BATTERY));
        }
        assertTrue(stalls.isEmpty(), "the debounce must require a continuous condition");
    }

    @Test
    void doesNotFireRepeatedlyWhileTheStallPersists() {
        runOneMotor(0, 3.0, dutyFor(30), 30.0, 9.0);
        assertEquals(1, stalls.size(),
                "one event per stall, not one per loop -- the listener would be flooded");
    }

    @Test
    void clearsWhenTheMotorRecoversAndCanFireAgainLater() {
        double t = runOneMotor(0, DEBOUNCE + 0.1, dutyFor(30), 30.0, 9.0);
        assertEquals(1, stalls.size());

        t = runOneMotor(t, 0.4, dutyFor(30), 30.0,
                detector.expectedCurrent(dutyFor(30), 30.0, BATTERY));
        assertEquals(List.of(0), cleared);
        assertFalse(detector.isStalled(0));

        runOneMotor(t, DEBOUNCE + 0.1, dutyFor(30), 30.0, 9.0);
        assertEquals(2, stalls.size(), "a fresh stall after recovery should fire again");
    }

    @Test
    void reportsEachMotorIndependently() {
        // All four are commanded the same; only the back-left one is wedged, and
        // only it draws anomalous current.
        double duty = dutyFor(30);
        double healthyCurrent = detector.expectedCurrent(duty, 30.0, BATTERY);
        for (double t = 0; t < DEBOUNCE + 0.1; t += 0.02) {
            detector.update(t,
                    new double[]{duty, duty, duty, duty},
                    new double[]{30.0, 30.0, 30.0, 30.0},
                    new double[]{healthyCurrent, healthyCurrent, 8.8, healthyCurrent});
        }

        assertEquals(1, stalls.size());
        assertEquals(2, stalls.get(0).motorIndex, "back-left is index 2");
        assertFalse(detector.isStalled(0));
        assertTrue(detector.isStalled(2));
    }

    @Test
    void theAbsoluteExcessFloorStopsCoastingFromReportingStalls() {
        // Barely commanded, barely moving: prediction is tiny, so a small
        // absolute reading is a large ratio and means nothing.
        double power = 0.02;
        double velocity = 1.0;
        double expected = detector.expectedCurrent(power, velocity, BATTERY);
        assertTrue(expected < 0.5, "prediction should be small here: " + expected);

        run(0, 2.0, power, velocity, expected * 3.0);

        assertTrue(stalls.isEmpty(),
                "3x a negligible prediction is still negligible");
    }

    @Test
    void predictionTracksTheTunedModelRatherThanACompiledConstant() {
        // The point of tying this to the feedforward: once the tuner learns the
        // robot needs more voltage per unit speed, more of a given command is
        // explained by back EMF and the same reading is less suspicious.
        double before = detector.expectedCurrent(0.6, 30.0, BATTERY);

        feedforward.setActiveImmediately(new FeedforwardGains(0.08, 0.0200, 0.0012));
        double after = detector.expectedCurrent(0.6, 30.0, BATTERY);

        assertTrue(after < before,
                "a higher kV means more of that command is explained by back EMF");
    }

    @Test
    void anImplausibleBatteryReadingSuppressesDetection() {
        StallDetector offline = StallDetector.builder(
                        feedforward, VoltageSource.constant(0.0))
                .debounceSeconds(0.0)
                .build();
        List<StallEvent> events = new ArrayList<>();
        offline.addListener(events::add);

        for (double t = 0; t < 1.0; t += 0.02) {
            offline.update(t, new double[]{1.0}, new double[]{0.0}, new double[]{20.0});
        }

        assertTrue(events.isEmpty(), "cannot judge current without a usable voltage");
    }

    @Test
    void resetForgetsDebounceState() {
        run(0, 0.15, dutyFor(30), 30.0, 9.0); // partway through the debounce
        detector.reset();
        run(0.15, 0.1, dutyFor(30), 30.0, 9.0);

        assertTrue(stalls.isEmpty(), "reset should restart the debounce");
    }

    @Test
    void nullInputIsIgnoredRatherThanThrowing() {
        detector.update(0, null, new double[]{1}, new double[]{1});
        detector.update(0, new double[]{1}, null, new double[]{1});
        detector.update(0, new double[]{1}, new double[]{1}, null);
        assertTrue(stalls.isEmpty());
    }

    @Test
    void invalidConfigurationIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> StallDetector.builder(feedforward, VoltageSource.constant(12))
                        .stallCurrentAmps(0).build());
        assertThrows(IllegalArgumentException.class,
                () -> StallDetector.builder(feedforward, VoltageSource.constant(12))
                        .currentRatioThreshold(1.0).build());
        assertThrows(IllegalArgumentException.class,
                () -> StallDetector.builder(feedforward, VoltageSource.constant(12))
                        .debounceSeconds(-1).build());
        assertThrows(IllegalArgumentException.class,
                () -> StallDetector.builder(null, VoltageSource.constant(12)));
    }
}
