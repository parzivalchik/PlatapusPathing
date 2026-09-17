package com.platapus.pathing;

import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.FeedforwardTuner;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class FeedforwardTunerTest {

    /** The robot the tuner is trying to discover. */
    private static final FeedforwardGains TRUTH = new FeedforwardGains(0.08, 0.0165, 0.0012);

    /** A deliberately wrong starting guess. */
    private static final FeedforwardGains GUESS = new FeedforwardGains(0.0, 0.030, 0.0);

    private FeedforwardTuner tuner() {
        return FeedforwardTuner.builder(GUESS).forgettingFactor(0.99).build();
    }

    /** The voltage the true model would apply for a given velocity and acceleration. */
    private static double trueCommand(double v, double a) {
        return TRUTH.predict(v, a);
    }

    /** Feeds a spread of driving conditions, as a real auto would produce. */
    private static void driveRealistically(FeedforwardTuner tuner, int samples, Random random,
                                           double noiseStdDev) {
        for (int i = 0; i < samples; i++) {
            double v = (random.nextDouble() * 2 - 1) * 50;
            if (Math.abs(v) < 3) {
                v = Math.copySign(3 + random.nextDouble() * 10, v);
            }
            double a = (random.nextDouble() * 2 - 1) * 40;
            double u = trueCommand(v, a) + random.nextGaussian() * noiseStdDev;
            tuner.addSample(u, v, a);
        }
    }

    @Test
    void recoversTheTrueGainsFromCleanData() {
        FeedforwardTuner tuner = tuner();
        driveRealistically(tuner, 3000, new Random(1), 0.0);

        FeedforwardGains fitted = tuner.getGains();
        assertEquals(TRUTH.kS, fitted.kS, 1e-3, "kS: " + fitted);
        assertEquals(TRUTH.kV, fitted.kV, 1e-4, "kV: " + fitted);
        assertEquals(TRUTH.kA, fitted.kA, 1e-4, "kA: " + fitted);
    }

    @Test
    void convergesDespiteMeasurementNoise() {
        FeedforwardTuner tuner = tuner();
        driveRealistically(tuner, 8000, new Random(7), 0.01);

        FeedforwardGains fitted = tuner.getGains();
        assertEquals(TRUTH.kV, fitted.kV, 2e-3, "kV: " + fitted);
        assertEquals(TRUTH.kS, fitted.kS, 0.05, "kS: " + fitted);
    }

    @Test
    void startsFromTheSuppliedGuessAndImprovesOnIt() {
        FeedforwardTuner tuner = tuner();
        assertEquals(GUESS.kV, tuner.getGains().kV, 1e-12);

        double initialError = Math.abs(tuner.getGains().kV - TRUTH.kV);
        driveRealistically(tuner, 2000, new Random(3), 0.005);
        double finalError = Math.abs(tuner.getGains().kV - TRUTH.kV);

        assertTrue(finalError < initialError / 10,
                "should get much closer: " + initialError + " -> " + finalError);
    }

    @Test
    void confidenceRisesAsTheFitBecomesDetermined() {
        FeedforwardTuner tuner = tuner();
        double before = tuner.getConfidence();
        driveRealistically(tuner, 2000, new Random(11), 0.0);
        assertTrue(tuner.getConfidence() > before,
                "confidence should rise with data: " + before + " -> " + tuner.getConfidence());
    }

    @Test
    void nearZeroVelocitySamplesAreRejected() {
        // sign(v) is the kS regressor and its sign is noise near zero.
        FeedforwardTuner tuner = FeedforwardTuner.builder(GUESS).minVelocity(2.0).build();

        assertFalse(tuner.addSample(0.05, 0.0, 0), "zero velocity");
        assertFalse(tuner.addSample(0.05, 1.5, 0), "below the floor");
        assertTrue(tuner.addSample(0.05, 5.0, 0), "above the floor");

        assertEquals(1, tuner.getSampleCount());
        assertEquals(2, tuner.getRejectedCount());
    }

    @Test
    void saturatedSamplesAreRejected() {
        // A motor at full power is not obeying the model; fitting to it teaches
        // the model that speed is cheap and biases kV down.
        FeedforwardTuner tuner = FeedforwardTuner.builder(GUESS).maxCommand(0.95).build();

        assertFalse(tuner.addSample(1.0, 40, 0));
        assertFalse(tuner.addSample(-0.99, -40, 0));
        assertTrue(tuner.addSample(0.5, 30, 0));
    }

    @Test
    void saturatedDataDoesNotBiasTheFitDownward() {
        FeedforwardTuner clean = tuner();
        FeedforwardTuner polluted = tuner();
        Random random = new Random(5);

        for (int i = 0; i < 3000; i++) {
            double v = 5 + random.nextDouble() * 45;
            double a = (random.nextDouble() * 2 - 1) * 30;
            double u = trueCommand(v, a);
            clean.addSample(u, v, a);
            polluted.addSample(u, v, a);

            // Interleave saturated rows: the robot asked for more than it got.
            polluted.addSample(1.0, 55 + random.nextDouble() * 30, 0);
        }

        assertEquals(clean.getGains().kV, polluted.getGains().kV, 1e-6,
                "saturated rows should have been filtered out entirely");
    }

    @Test
    void nonFiniteInputIsRejectedRatherThanPoisoningTheFit() {
        FeedforwardTuner tuner = tuner();
        driveRealistically(tuner, 500, new Random(2), 0.0);
        FeedforwardGains before = tuner.getGains();

        assertFalse(tuner.addSample(Double.NaN, 20, 0));
        assertFalse(tuner.addSample(0.3, Double.NaN, 0));
        assertFalse(tuner.addSample(0.3, 20, Double.POSITIVE_INFINITY));

        assertEquals(before, tuner.getGains(), "a bad row must not move the fit");
    }

    @Test
    void forgettingLetsTheFitTrackARobotThatChanges() {
        // A belt tightens mid-match and the robot now needs more voltage for the
        // same speed. A fit with no forgetting would average the two regimes.
        FeedforwardTuner tuner = FeedforwardTuner.builder(GUESS)
                .forgettingFactor(0.98).build();
        Random random = new Random(13);

        driveRealistically(tuner, 3000, random, 0.0);
        assertEquals(TRUTH.kV, tuner.getGains().kV, 1e-3);

        FeedforwardGains changed = new FeedforwardGains(0.08, 0.0220, 0.0012);
        for (int i = 0; i < 4000; i++) {
            double v = (random.nextDouble() * 2 - 1) * 50;
            if (Math.abs(v) < 3) v = Math.copySign(6, v);
            double a = (random.nextDouble() * 2 - 1) * 30;
            tuner.addSample(changed.predict(v, a), v, a);
        }

        assertEquals(changed.kV, tuner.getGains().kV, 1e-3,
                "should have tracked the change, got " + tuner.getGains());
    }

    @Test
    void aHigherForgettingFactorTracksChangesMoreSlowly() {
        FeedforwardGains changed = new FeedforwardGains(0.08, 0.0220, 0.0012);

        double fastError = trackingErrorAfterChange(0.98, changed);
        double slowError = trackingErrorAfterChange(0.9995, changed);

        assertTrue(fastError < slowError,
                "lower lambda should adapt faster: " + fastError + " vs " + slowError);
    }

    private double trackingErrorAfterChange(double lambda, FeedforwardGains changed) {
        FeedforwardTuner tuner = FeedforwardTuner.builder(GUESS)
                .forgettingFactor(lambda).build();
        Random random = new Random(21);
        driveRealistically(tuner, 3000, random, 0.0);

        for (int i = 0; i < 300; i++) {
            double v = (random.nextDouble() * 2 - 1) * 50;
            if (Math.abs(v) < 3) v = Math.copySign(6, v);
            double a = (random.nextDouble() * 2 - 1) * 30;
            tuner.addSample(changed.predict(v, a), v, a);
        }
        return Math.abs(tuner.getGains().kV - changed.kV);
    }

    @Test
    void residualFallsAsTheModelStartsExplainingTheData() {
        FeedforwardTuner tuner = tuner();
        driveRealistically(tuner, 40, new Random(17), 0.0);
        double early = tuner.getRmsResidual();

        FeedforwardTuner settled = tuner();
        driveRealistically(settled, 4000, new Random(17), 0.0);
        double late = settled.getRmsResidual();

        assertTrue(late < early,
                "residual should shrink as the fit improves: " + early + " -> " + late);
    }

    @Test
    void resetRestartsTheFitFromGivenGains() {
        FeedforwardTuner tuner = tuner();
        driveRealistically(tuner, 1000, new Random(23), 0.0);
        assertTrue(tuner.getSampleCount() > 0);

        FeedforwardGains fresh = new FeedforwardGains(0.01, 0.02, 0.0);
        tuner.reset(fresh, 1e-3);

        assertEquals(fresh, tuner.getGains());
        assertEquals(0, tuner.getSampleCount());
    }

    @Test
    void invalidConfigurationIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> FeedforwardTuner.builder(GUESS).forgettingFactor(0).build());
        assertThrows(IllegalArgumentException.class,
                () -> FeedforwardTuner.builder(GUESS).forgettingFactor(1.5).build());
        assertThrows(IllegalArgumentException.class,
                () -> FeedforwardTuner.builder(GUESS).initialCovariance(-1).build());
        assertThrows(IllegalArgumentException.class, () -> FeedforwardTuner.builder(null));
    }
}
