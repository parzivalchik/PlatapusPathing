package com.platapus.pathing;

import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.kinematics.Kinematics;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.localization.FusedLocalizer;
import com.platapus.pathing.localization.VisionPoseSource;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class FusedLocalizerTest {

    private MecanumKinematics kinematics;
    private SimulatedRobot.ManualClock clock;
    private Encoders encoders;

    @BeforeEach
    void setUp() {
        kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        clock = new SimulatedRobot.ManualClock();
        encoders = new Encoders(kinematics);
    }

    private static final class Encoders implements Drivetrain {
        private final Kinematics kinematics;
        private final double[] positions;

        Encoders(Kinematics k) {
            this.kinematics = k;
            this.positions = new double[k.getWheelCount()];
        }

        void advance(double[] deltas) {
            for (int i = 0; i < positions.length; i++) positions[i] += deltas[i];
        }

        /** Drives forward, in inches, without touching heading. */
        void forward(double inches) {
            advance(new double[]{inches, inches, inches, inches});
        }

        @Override public Kinematics getKinematics() { return kinematics; }
        @Override public void setWheelPowers(double[] powers) { }
        @Override public double[] getWheelPositions() { return positions.clone(); }
    }

    /** A vision source the test hands observations to directly. */
    private static final class FakeVision implements VisionPoseSource {
        Observation next;
        @Override public Observation getObservation() { return next; }
    }

    @Test
    void behavesLikeOdometryWhenNothingElseIsAvailable() {
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock).build();
        localizer.update();

        for (int i = 0; i < 20; i++) {
            encoders.forward(1.0);
            clock.advance(0.02);
            localizer.update();
        }

        assertEquals(20.0, localizer.getPose().getX(), 1e-6);
        assertEquals(0.0, localizer.getPose().getY(), 1e-6);
    }

    @Test
    void headingSourceIsZeroedAtTheFirstUpdate() {
        double[] gyro = {2.4};
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .headingSource(() -> gyro[0]).build();
        localizer.update();

        assertEquals(0.0, localizer.getPose().getHeading(), 1e-9);
    }

    @Test
    void gyroCorrectsWheelDerivedHeadingDrift() {
        // The wheels claim a steady rotation; the gyro says the robot is
        // straight. Over many loops the filter should side with the gyro.
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .headingSource(() -> 0.0)
                .headingVariance(1e-6)
                .build();
        localizer.update();

        for (int i = 0; i < 200; i++) {
            encoders.advance(new double[]{-0.05, 0.05, -0.05, 0.05}); // pure spin
            clock.advance(0.02);
            localizer.update();
        }

        assertEquals(0.0, localizer.getPose().getHeading(), Math.toRadians(1),
                "gyro should hold heading against drifting wheels");
    }

    @Test
    void confidenceDecaysWhileDrivingAndRecoversOnAVisionFix() {
        FakeVision vision = new FakeVision();
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .visionSource(vision)
                .build();
        localizer.update();

        for (int i = 0; i < 300; i++) {
            encoders.advance(new double[]{0.4, 0.4, 0.5, 0.5});
            clock.advance(0.02);
            localizer.update();
        }
        double drifted = localizer.getConfidence();
        assertTrue(drifted < 1.0, "confidence should fall while driving blind");

        vision.next = new VisionPoseSource.Observation(
                localizer.getPose(), new double[]{0.05, 0.05, 0.005}, clock.seconds());
        clock.advance(0.02);
        localizer.update();

        assertTrue(localizer.getConfidence() > drifted,
                "a good fix should restore confidence");
    }

    @Test
    void visionNudgesRatherThanTeleports() {
        FakeVision vision = new FakeVision();
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .visionSource(vision).build();
        localizer.update();

        for (int i = 0; i < 100; i++) {
            encoders.forward(0.3);
            clock.advance(0.02);
            localizer.update();
        }

        Pose2d before = localizer.getPose();
        Pose2d claimed = new Pose2d(before.getX() + 12, before.getY(), before.getHeading());

        vision.next = new VisionPoseSource.Observation(
                claimed, new double[]{4.0, 4.0, 0.5}, clock.seconds());
        clock.advance(0.02);
        localizer.update();

        Pose2d after = localizer.getPose();
        double moved = after.getX() - before.getX();
        assertTrue(moved > 0, "should move toward a plausible observation");
        assertTrue(moved < 6.0,
                "a vague 12\" correction should not be swallowed whole, moved " + moved);
    }

    @Test
    void aStaleVisionObservationIsAppliedOnlyOnce() {
        // Camera pipelines run slower than the control loop and re-serve the
        // same detection. Folding it in repeatedly would fake certainty.
        FakeVision vision = new FakeVision();
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .visionSource(vision).build();
        localizer.update();

        for (int i = 0; i < 100; i++) {
            encoders.forward(0.3);
            clock.advance(0.02);
            localizer.update();
        }

        Pose2d before = localizer.getPose();
        vision.next = new VisionPoseSource.Observation(
                new Pose2d(before.getX() + 10, before.getY(), before.getHeading()),
                new double[]{1.0, 1.0, 0.1}, clock.seconds());

        clock.advance(0.02);
        localizer.update();
        Pose2d afterFirst = localizer.getPose();

        // Same observation, same timestamp, served again for many loops.
        for (int i = 0; i < 50; i++) {
            clock.advance(0.02);
            localizer.update();
        }
        Pose2d afterRepeats = localizer.getPose();

        assertEquals(afterFirst.getX(), afterRepeats.getX(), 1e-9,
                "a repeated observation must not keep pulling the estimate");
    }

    @Test
    void aNewerVisionObservationIsApplied() {
        FakeVision vision = new FakeVision();
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .visionSource(vision).build();
        localizer.update();
        for (int i = 0; i < 50; i++) {
            encoders.forward(0.3);
            clock.advance(0.02);
            localizer.update();
        }

        Pose2d target = new Pose2d(30, 4, 0.1);
        double initialGap = localizer.getPose().position.distanceTo(target.position);
        double previousGap = initialGap;
        double moved = 0;

        for (int i = 0; i < 40; i++) {
            clock.advance(0.02);
            vision.next = new VisionPoseSource.Observation(
                    target, new double[]{0.2, 0.2, 0.02}, clock.seconds());
            Pose2d before = localizer.getPose();
            localizer.update();
            moved += before.position.distanceTo(localizer.getPose().position);

            double gap = localizer.getPose().position.distanceTo(target.position);
            assertTrue(gap <= previousGap + 1e-9,
                    "each fresh fix should close the gap, not reopen it (step " + i + ")");
            previousGap = gap;
        }

        assertTrue(moved > 0, "fresh observations should keep being applied");
        assertTrue(previousGap < initialGap * 0.5,
                "repeated fresh fixes should close most of the gap: "
                        + initialGap + "\" -> " + previousGap + "\"");
    }

    @Test
    void setPoseRebasesTheFilterAndTheGyroOffset() {
        double[] gyro = {0.7};
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .headingSource(() -> gyro[0]).build();
        localizer.update();

        localizer.setPose(new Pose2d(12, -8, Math.PI / 2));
        assertEquals(12.0, localizer.getPose().getX(), 1e-9);
        assertEquals(Math.PI / 2, localizer.getPose().getHeading(), 1e-9);
        assertEquals(1.0, localizer.getConfidence(), 0.01);

        clock.advance(0.02);
        localizer.update();
        assertEquals(Math.PI / 2, localizer.getPose().getHeading(), 1e-3,
                "a stationary gyro should not move the heading after a rebase");
    }

    @Test
    void isADropInReplacementForTheEncoderLocalizer() {
        // Same interface, same start-relative convention, so the follower cannot
        // tell them apart. Worth asserting so the substitution stays true.
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock).build();
        localizer.update();
        encoders.forward(5);
        clock.advance(0.02);
        localizer.update();

        assertEquals(5.0, localizer.getPose().getX(), 1e-6);
        assertEquals(250.0, localizer.getVelocity().vx, 1e-6); // 5" in 0.02s
        assertEquals(5.0, localizer.getLastTwist().dx, 1e-6);
    }
}
