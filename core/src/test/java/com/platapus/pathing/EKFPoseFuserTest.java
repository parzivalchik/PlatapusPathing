package com.platapus.pathing;

import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.localization.EKFPoseFuser;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class EKFPoseFuserTest {

    private EKFPoseFuser fuser() {
        return EKFPoseFuser.builder().build();
    }

    @Test
    void predictionAloneMatchesPlainOdometry() {
        // With no measurements, the filter must not be worse than dead
        // reckoning: its mean is exactly the odometry integration.
        EKFPoseFuser f = fuser();
        Pose2d expected = Pose2d.ZERO;

        for (int i = 0; i < 50; i++) {
            Twist2d twist = new Twist2d(0.2, 0.01, 0.02);
            f.predict(twist, 0.02);
            expected = twist.applyTo(expected);
        }

        assertTrue(f.getPose().epsilonEquals(expected, 1e-9, 1e-9),
                "filter " + f.getPose() + " vs odometry " + expected);
    }

    @Test
    void uncertaintyGrowsWhileDrivingBlind() {
        EKFPoseFuser f = fuser();
        double before = f.getPositionStdDev();

        for (int i = 0; i < 100; i++) {
            f.predict(new Twist2d(0.5, 0, 0), 0.02);
        }

        assertTrue(f.getPositionStdDev() > before,
                "covariance should grow with distance travelled");
        assertTrue(f.getConfidence() < 1.0);
    }

    @Test
    void uncertaintyGrowsFasterSidewaysThanForwards() {
        // Mecanum scrubs laterally and the encoders cannot see it. The filter is
        // told that, and it has to actually show up in the covariance.
        EKFPoseFuser forwards = fuser();
        EKFPoseFuser sideways = fuser();

        for (int i = 0; i < 50; i++) {
            forwards.predict(new Twist2d(0.5, 0, 0), 0.02);
            sideways.predict(new Twist2d(0, 0.5, 0), 0.02);
        }

        assertTrue(sideways.getPositionStdDev() > forwards.getPositionStdDev(),
                "lateral travel should be trusted less than forward travel");
    }

    @Test
    void headingMeasurementPullsTheEstimateTowardIt() {
        EKFPoseFuser f = fuser();
        // Accumulate some heading uncertainty first, or the filter rightly
        // ignores the measurement.
        for (int i = 0; i < 100; i++) {
            f.predict(new Twist2d(0.2, 0, 0.01), 0.02);
        }

        double before = f.getPose().heading;
        double measured = before + Math.toRadians(5);
        f.correctHeading(measured, 1e-4);
        double after = f.getPose().heading;

        assertTrue(after > before, "estimate should move toward the measurement");
        assertTrue(after < measured, "and should not jump all the way to it");
    }

    @Test
    void headingMeasurementShrinksHeadingUncertainty() {
        EKFPoseFuser f = fuser();
        for (int i = 0; i < 100; i++) {
            f.predict(new Twist2d(0.2, 0, 0.01), 0.02);
        }

        double before = f.getHeadingStdDev();
        f.correctHeading(f.getPose().heading, 1e-4);
        assertTrue(f.getHeadingStdDev() < before, "a measurement must reduce uncertainty");
    }

    @Test
    void aTrustedMeasurementMovesTheEstimateMoreThanAnUntrustedOne() {
        // This is the whole reason for using a filter instead of resetPose().
        double offset = Math.toRadians(10);

        EKFPoseFuser trusting = fuser();
        EKFPoseFuser sceptical = fuser();
        for (int i = 0; i < 100; i++) {
            trusting.predict(new Twist2d(0.2, 0, 0.01), 0.02);
            sceptical.predict(new Twist2d(0.2, 0, 0.01), 0.02);
        }

        double base = trusting.getPose().heading;
        trusting.correctHeading(base + offset, 1e-6);   // tight measurement
        sceptical.correctHeading(base + offset, 1e-1);  // vague measurement

        double moveTrusted = trusting.getPose().heading - base;
        double moveSceptical = sceptical.getPose().heading - base;

        assertTrue(moveTrusted > moveSceptical * 5,
                "a tight measurement should dominate a vague one: "
                        + moveTrusted + " vs " + moveSceptical);
    }

    @Test
    void headingCorrectionAlsoNudgesPositionThroughCorrelation() {
        // A robot that drove a curve has correlated position and heading error.
        // Learning the heading therefore says something about position too --
        // information a resetPose()-style override throws away entirely.
        EKFPoseFuser f = fuser();
        for (int i = 0; i < 200; i++) {
            f.predict(new Twist2d(0.3, 0, 0.01), 0.02);
        }

        Pose2d before = f.getPose();
        f.correctHeading(before.heading + Math.toRadians(8), 1e-5);
        Pose2d after = f.getPose();

        assertTrue(before.position.distanceTo(after.position) > 1e-6,
                "position should shift when a correlated heading is corrected");
    }

    @Test
    void visionUpdateMovesTowardTheObservationWithoutTeleporting() {
        EKFPoseFuser f = fuser();
        for (int i = 0; i < 200; i++) {
            f.predict(new Twist2d(0.3, 0.02, 0.005), 0.02);
        }

        Pose2d before = f.getPose();
        Pose2d observed = new Pose2d(before.getX() + 6, before.getY() - 3, before.getHeading());
        f.correctPose(observed, new double[]{0.5, 0.5, 0.05});
        Pose2d after = f.getPose();

        double movedToward = before.position.distanceTo(after.position);
        double gap = before.position.distanceTo(observed.position);
        assertTrue(movedToward > 0, "should move toward the observation");
        assertTrue(movedToward < gap, "should not snap onto it");
    }

    /**
     * Drives blind, then feeds the same observation repeatedly at the given
     * variance. Returns the fraction of the initial gap that was closed, and
     * asserts the approach never reverses.
     */
    private double closedFractionUnderRepeatedVision(double variance) {
        EKFPoseFuser f = fuser();
        for (int i = 0; i < 100; i++) {
            f.predict(new Twist2d(0.3, 0, 0), 0.02);
        }

        Pose2d truth = new Pose2d(20, 5, 0.2);
        double initialGap = f.getPose().position.distanceTo(truth.position);
        double previous = initialGap;

        for (int i = 0; i < 60; i++) {
            f.predict(new Twist2d(0, 0, 0), 0.02);
            f.correctPose(truth, new double[]{variance, variance, variance / 25});

            double gap = f.getPose().position.distanceTo(truth.position);
            assertTrue(gap <= previous + 1e-12,
                    "approach reversed at step " + i + ": " + previous + " -> " + gap);
            previous = gap;
        }
        return 1.0 - previous / initialGap;
    }

    @Test
    void repeatedConsistentVisionConvergesMonotonicallyTowardTheObservation() {
        double closed = closedFractionUnderRepeatedVision(0.25);
        assertTrue(closed > 0.5,
                "should close most of the gap, closed " + (closed * 100) + "%");
    }

    @Test
    void howFastVisionConvergesDependsOnHowMuchItIsTrusted() {
        // The property that actually matters. It converges slowly against a
        // vague fix and quickly against a tight one, because the filter is
        // weighing the observation against its own odometry rather than
        // obeying it. A resetPose()-style override would close 100% every time,
        // regardless of whether the observation deserved it.
        double vague = closedFractionUnderRepeatedVision(0.25);
        double sharp = closedFractionUnderRepeatedVision(0.01);

        assertTrue(sharp > vague,
                "a tighter fix should converge further: " + sharp + " vs " + vague);
        assertTrue(sharp > 0.9, "a very tight fix should nearly converge, got " + sharp);
        assertTrue(vague < 0.85, "a vague fix should not be swallowed whole, got " + vague);
    }

    @Test
    void confidenceFallsAsUncertaintyGrowsAndRecoversOnAMeasurement() {
        EKFPoseFuser f = fuser();
        assertEquals(1.0, f.getConfidence(), 0.01);

        for (int i = 0; i < 400; i++) {
            f.predict(new Twist2d(0.5, 0.1, 0.01), 0.02);
        }
        double drifted = f.getConfidence();
        assertTrue(drifted < 0.99, "confidence should decay while driving blind: " + drifted);

        f.correctPose(f.getPose(), new double[]{0.01, 0.01, 0.001});
        assertTrue(f.getConfidence() > drifted, "a good fix should restore confidence");
        assertTrue(f.getConfidence() <= 1.0);
    }

    @Test
    void covarianceStaysSymmetricAndPositiveOverALongRun() {
        // Asymmetry creeping in through floating point is the classic way an EKF
        // quietly goes unstable, so this is worth asserting explicitly.
        EKFPoseFuser f = fuser();
        for (int i = 0; i < 5000; i++) {
            f.predict(new Twist2d(0.2, 0.05, 0.02), 0.02);
            if (i % 3 == 0) f.correctHeading(f.getPose().heading + 0.001, 1e-4);
            if (i % 50 == 0) f.correctPose(f.getPose(), new double[]{1.0, 1.0, 0.1});

            double[] var = f.getVariance();
            for (int k = 0; k < 3; k++) {
                assertTrue(var[k] > 0, "variance went non-positive at step " + i);
                assertTrue(Double.isFinite(var[k]), "variance diverged at step " + i);
            }
        }
        assertTrue(f.getConfidence() > 0 && f.getConfidence() <= 1.0);
    }

    @Test
    void headingWrapIsHandledInTheInnovation() {
        EKFPoseFuser f = EKFPoseFuser.builder()
                .initialPose(new Pose2d(0, 0, Math.toRadians(179)))
                .initialVariance(1e-4, 1e-4, 0.05)
                .build();

        // Measuring -179 deg is a 2 degree move forward, not 358 back.
        f.correctHeading(Math.toRadians(-179), 1e-4);
        double heading = Math.toDegrees(f.getPose().heading);
        assertTrue(heading > 179 || heading < -179,
                "should cross the wrap, ended at " + heading + " deg");
    }

    @Test
    void resetClearsUncertaintyAsWellAsPose() {
        EKFPoseFuser f = fuser();
        for (int i = 0; i < 300; i++) {
            f.predict(new Twist2d(0.5, 0.2, 0.02), 0.02);
        }
        assertTrue(f.getConfidence() < 0.99);

        f.reset(new Pose2d(5, 5, 1.0));
        assertEquals(5.0, f.getPose().getX(), 1e-9);
        assertEquals(1.0, f.getConfidence(), 0.01);
    }

    @Test
    void badVariancesAreRejectedRatherThanSilentlyAbsorbed() {
        EKFPoseFuser f = fuser();
        assertThrows(IllegalArgumentException.class, () -> f.correctHeading(0, 0));
        assertThrows(IllegalArgumentException.class, () -> f.correctHeading(0, -1));
        assertThrows(IllegalArgumentException.class,
                () -> f.correctPose(Pose2d.ZERO, new double[]{1, 1, 0}));
        assertThrows(IllegalArgumentException.class,
                () -> f.correctPose(Pose2d.ZERO, new double[]{1, 1}));
        assertThrows(IllegalArgumentException.class, () -> f.predict(null, 0.02));
    }
}
