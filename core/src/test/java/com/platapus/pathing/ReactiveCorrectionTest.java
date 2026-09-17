package com.platapus.pathing;

import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Phase 2: the error-scaled reactive blend on top of the profile follower. */
class ReactiveCorrectionTest {

    private static final double DT = 0.02;
    private static final double MAX_WHEEL_VELOCITY = 60.0;

    private MecanumKinematics kinematics() {
        return MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(MAX_WHEEL_VELOCITY).build();
    }

    private FollowerConstants.Builder base() {
        return FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(30)
                .maxAcceleration(40)
                .kV(1.0 / MAX_WHEEL_VELOCITY)
                .translationalPID(2.0, 0.0, 0.0)
                .headingPID(3.0, 0.0, 0.0)
                .positionTolerance(1.0)
                .headingTolerance(Math.toRadians(3));
    }

    // --- the blend function itself, as pure math -----------------------------

    @Test
    void authorityIsOneWhenPerfectlyOnPath() {
        FollowerConstants c = base().reactiveErrorScale(4).reactiveAuthority(2.5).build();
        assertEquals(1.0, c.correctionAuthority(0.0, 1.0), 1e-9);
    }

    @Test
    void authorityRampsLinearlyToTheConfiguredMaximum() {
        FollowerConstants c = base().reactiveErrorScale(4).reactiveAuthority(3.0).build();
        assertEquals(1.0, c.correctionAuthority(0, 1.0), 1e-9);
        assertEquals(1.5, c.correctionAuthority(1, 1.0), 1e-9);
        assertEquals(2.0, c.correctionAuthority(2, 1.0), 1e-9);
        assertEquals(3.0, c.correctionAuthority(4, 1.0), 1e-9);
    }

    @Test
    void authoritySaturatesRatherThanGrowingWithoutBound() {
        FollowerConstants c = base().reactiveErrorScale(4).reactiveAuthority(3.0).build();
        assertEquals(3.0, c.correctionAuthority(40, 1.0), 1e-9);
        assertEquals(3.0, c.correctionAuthority(4000, 1.0), 1e-9);
    }

    @Test
    void authorityIsContinuousAcrossItsWholeRange() {
        // The reason for a blend rather than a threshold: no step change in
        // commanded velocity at the error where the robot is already struggling.
        FollowerConstants c = base().reactiveErrorScale(4).reactiveAuthority(2.5).build();
        double previous = c.correctionAuthority(0, 1.0);
        for (double e = 0; e <= 10; e += 0.01) {
            double a = c.correctionAuthority(e, 1.0);
            assertTrue(Math.abs(a - previous) < 0.02, "jump at error " + e);
            assertTrue(a >= previous - 1e-12, "authority should be monotonic at " + e);
            previous = a;
        }
    }

    @Test
    void authorityOfOneDisablesReactiveBlendingEntirely() {
        FollowerConstants c = base().reactiveAuthority(1.0).build();
        for (double e = 0; e <= 20; e += 1.0) {
            assertEquals(1.0, c.correctionAuthority(e, 1.0), 1e-9);
        }
    }

    @Test
    void lowConfidenceScalesAuthorityBackButNeverBelowTheFloor() {
        FollowerConstants c = base()
                .reactiveErrorScale(4).reactiveAuthority(2.0)
                .minConfidenceAuthority(0.35)
                .build();

        double trusted = c.correctionAuthority(4, 1.0);
        double doubted = c.correctionAuthority(4, 0.0);

        assertEquals(2.0, trusted, 1e-9);
        assertEquals(2.0 * 0.35, doubted, 1e-9);
        assertTrue(doubted > 0, "an untrusted estimate must still correct somewhat");
    }

    @Test
    void errorScaleIsUnitAware() {
        FollowerConstants metric = FollowerConstants.builder(DistanceUnit.CM)
                .maxVelocity(76.2).maxAcceleration(101.6).kV(0.017)
                .reactiveErrorScale(10.16)  // 4 inches
                .reactiveAuthority(3.0)
                .build();
        assertEquals(4.0, metric.getReactiveErrorScale(), 1e-9);
        assertEquals(3.0, metric.correctionAuthority(4.0, 1.0), 1e-9);
    }

    @Test
    void invalidBlendSettingsAreRejected() {
        assertThrows(IllegalStateException.class, () -> base().reactiveAuthority(0.5).build());
        assertThrows(IllegalStateException.class, () -> base().reactiveErrorScale(0).build());
        assertThrows(IllegalStateException.class,
                () -> base().minConfidenceAuthority(1.5).build());
    }

    // --- end to end on the simulated robot -----------------------------------

    /** Displaces the robot mid-path and returns how long it takes to get back. */
    private double recoveryTime(FollowerConstants constants, double displacement) {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        SimulatedRobot robot = new SimulatedRobot(kinematics(), clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants, clock);

        follower.followPath(PathChain.of(
                new BezierLine(new Point(0, 0), new Point(72, 0))));

        for (int i = 0; i < 25; i++) {
            follower.update();
            robot.step(DT);
        }
        robot.displace(0, displacement, 0);

        double elapsed = 0;
        while (follower.isBusy() && elapsed < 20) {
            follower.update();
            robot.step(DT);
            elapsed += DT;
            if (Math.abs(robot.getTruePose().getY()) < 0.5) return elapsed;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Test
    void reactiveBlendingRecoversFasterFromALargeDisturbance() {
        FollowerConstants off = base().reactiveAuthority(1.0).build();
        FollowerConstants on = base().reactiveErrorScale(4).reactiveAuthority(3.0).build();

        double withoutReactive = recoveryTime(off, 8.0);
        double withReactive = recoveryTime(on, 8.0);

        assertTrue(Double.isFinite(withReactive), "reactive follower failed to recover");
        assertTrue(withReactive < withoutReactive,
                "reactive should recover sooner: " + withReactive + "s vs "
                        + withoutReactive + "s");
    }

    @Test
    void reactiveBlendingDoesNotDisturbNormalOnPathTracking() {
        // The point of blending on error is that a well-tracking robot is
        // unaffected. Both configurations should drive the same clean path.
        FollowerConstants off = base().reactiveAuthority(1.0).build();
        FollowerConstants on = base().reactiveErrorScale(4).reactiveAuthority(3.0).build();

        double worstOff = worstCrossTrack(off);
        double worstOn = worstCrossTrack(on);

        assertTrue(worstOff < 0.5, "baseline should track cleanly, was " + worstOff);
        assertTrue(worstOn < 0.5, "reactive should track just as cleanly, was " + worstOn);
    }

    private double worstCrossTrack(FollowerConstants constants) {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        SimulatedRobot robot = new SimulatedRobot(kinematics(), clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants, clock);
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0))));

        double worst = 0;
        while (follower.isBusy()) {
            follower.update();
            robot.step(DT);
            worst = Math.max(worst, Math.abs(robot.getTruePose().getY()));
        }
        return worst;
    }

    @Test
    void reportedAuthorityRisesWhenTheRobotIsKnockedOff() {
        FollowerConstants constants = base()
                .reactiveErrorScale(4).reactiveAuthority(3.0).build();

        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        SimulatedRobot robot = new SimulatedRobot(kinematics(), clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants, clock);
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(72, 0))));

        for (int i = 0; i < 25; i++) {
            follower.update();
            robot.step(DT);
        }
        double onPath = follower.getCorrectionAuthority();

        robot.displace(0, 10, 0);
        follower.update();
        double knockedOff = follower.getCorrectionAuthority();

        assertTrue(onPath < 1.3, "should be near 1 while tracking, was " + onPath);
        assertEquals(3.0, knockedOff, 1e-6, "should saturate when far off path");
    }

    @Test
    void correctionIsCappedSoItCannotSqueezeOutTheFeedforward() {
        FollowerConstants constants = base()
                .reactiveErrorScale(2).reactiveAuthority(4.0)
                .maxCorrectionVelocity(10)
                .build();

        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        SimulatedRobot robot = new SimulatedRobot(kinematics(), clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants, clock);
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(72, 0))));

        for (int i = 0; i < 25; i++) {
            follower.update();
            robot.step(DT);
        }

        // A 40" displacement times kP 2.0 times authority 4 would ask for 320"/s
        // without the cap -- ten times what the robot can do.
        robot.displace(0, 40, 0);
        follower.update();

        for (double p : robot.getLastPowers()) {
            assertTrue(Math.abs(p) <= 1.0 + 1e-9, "power out of range: " + p);
            assertTrue(Double.isFinite(p));
        }

        // The robot must still be making forward progress, not just crabbing.
        assertTrue(robot.getLastPowers()[0] != robot.getLastPowers()[1],
                "expected a mixed command, not a pure strafe");
    }

    @Test
    void followerStillFinishesNormallyWithReactiveBlendingOn() {
        FollowerConstants constants = base()
                .reactiveErrorScale(4).reactiveAuthority(3.0).build();

        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        SimulatedRobot robot = new SimulatedRobot(kinematics(), clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants, clock);
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(36, 0))));

        double elapsed = 0;
        while (follower.isBusy() && elapsed < 15) {
            follower.update();
            robot.step(DT);
            elapsed += DT;
        }

        assertTrue(follower.isAtTarget());
        assertEquals(36.0, robot.getTruePose().getX(), 1.0);
    }
}
