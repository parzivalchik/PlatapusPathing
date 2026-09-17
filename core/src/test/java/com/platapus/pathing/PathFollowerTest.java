package com.platapus.pathing;

import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PathFollowerTest {

    private static final double DT = 0.02; // 50 Hz, a realistic FTC loop
    private static final double MAX_WHEEL_VELOCITY = 60.0;

    private MecanumKinematics kinematics;
    private SimulatedRobot.ManualClock clock;
    private SimulatedRobot robot;
    private PathFollower follower;

    @BeforeEach
    void setUp() {
        kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(MAX_WHEEL_VELOCITY)
                .build();
        clock = new SimulatedRobot.ManualClock();
        robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);
        follower = new PathFollower(robot, robot, defaultConstants(), clock);
    }

    private FollowerConstants defaultConstants() {
        return FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(30)
                .maxAcceleration(40)
                .kV(1.0 / MAX_WHEEL_VELOCITY)
                .translationalPID(2.0, 0.0, 0.0)
                .headingPID(3.0, 0.0, 0.0)
                .positionTolerance(1.0)
                .headingTolerance(Math.toRadians(3))
                .build();
    }

    /** Runs the follower to completion, or until {@code maxSeconds} elapses. */
    private double runToCompletion(double maxSeconds) {
        double elapsed = 0;
        while (follower.isBusy() && elapsed < maxSeconds) {
            follower.update();
            robot.step(DT);
            elapsed += DT;
        }
        return elapsed;
    }

    @Test
    void followerStartsIdleAndBecomesBusyOnlyOnceGivenAPath() {
        assertFalse(follower.isBusy());
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(24, 0))));
        assertTrue(follower.isBusy());
    }

    @Test
    void drivesAStraightLineToWithinTolerance() {
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(24, 0))));
        runToCompletion(10);

        assertFalse(follower.isBusy());
        assertTrue(follower.isAtTarget(),
                "ended " + follower.getPositionError() + "\" away");
        assertEquals(24.0, robot.getTruePose().getX(), 1.0);
        assertEquals(0.0, robot.getTruePose().getY(), 1.0);
    }

    @Test
    void strafesSideways() {
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(0, 24))));
        runToCompletion(10);

        assertTrue(follower.isAtTarget());
        assertEquals(24.0, robot.getTruePose().getY(), 1.0);
    }

    @Test
    void followsACurveAndHoldsAConstantHeadingThroughout() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(0, 0), new Point(24, 0), new Point(24, 24)))
                .setConstantHeadingInterpolation(0)
                .build();
        follower.followPath(chain);

        double worstHeading = 0;
        while (follower.isBusy()) {
            follower.update();
            robot.step(DT);
            worstHeading = Math.max(worstHeading, Math.abs(robot.getTruePose().getHeading()));
        }

        assertTrue(follower.isAtTarget());
        assertEquals(24.0, robot.getTruePose().getX(), 1.5);
        assertEquals(24.0, robot.getTruePose().getY(), 1.5);
        assertTrue(worstHeading < Math.toRadians(10),
                "heading should stay near 0, drifted to " + Math.toDegrees(worstHeading));
    }

    @Test
    void turnsWhileDrivingWithLinearHeadingInterpolation() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                .build();
        follower.followPath(chain);
        runToCompletion(10);

        assertTrue(follower.isAtTarget());
        assertEquals(Math.toRadians(90), robot.getTruePose().getHeading(), Math.toRadians(5));
    }

    @Test
    void followsAMultiSegmentChainThroughEverySegment() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
                .addPath(new BezierLine(new Point(24, 0), new Point(24, 24)))
                .addPath(new BezierLine(new Point(24, 24), new Point(0, 24)))
                .build();
        follower.followPath(chain);

        boolean[] visited = new boolean[3];
        while (follower.isBusy()) {
            follower.update();
            visited[follower.getProjectedState().segmentIndex] = true;
            robot.step(DT);
        }

        for (int i = 0; i < visited.length; i++) {
            assertTrue(visited[i], "never reached segment " + i);
        }
        assertEquals(0.0, robot.getTruePose().getX(), 2.0);
        assertEquals(24.0, robot.getTruePose().getY(), 2.0);
    }

    @Test
    void pathTakesRoughlyTheTimeTheProfilePredicts() {
        PathChain chain = PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0)));
        follower.followPath(chain);
        double predicted = follower.getProfile().duration();
        double actual = runToCompletion(15);

        // The follower may spend a little extra time settling, but should not
        // take dramatically longer than the profile says.
        assertTrue(actual >= predicted - DT,
                "finished implausibly early: " + actual + "s vs " + predicted + "s");
        assertTrue(actual < predicted + 1.0,
                "took " + actual + "s for a " + predicted + "s profile");
    }

    @Test
    void repeatingTheSamePathTakesTheSameTime() {
        // Time-consistency is the point of profiling: two identical runs should
        // take the same number of loops.
        PathChain chain = PathChain.of(new BezierLine(new Point(0, 0), new Point(36, 0)));

        follower.followPath(chain);
        double first = runToCompletion(15);

        setUp();
        follower.followPath(chain);
        double second = runToCompletion(15);

        assertEquals(first, second, DT * 2);
    }

    @Test
    void recoversAfterBeingShovedOffThePath() {
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0))));

        // Drive a while, then knock the robot 6" sideways.
        for (int i = 0; i < 30; i++) {
            follower.update();
            robot.step(DT);
        }
        robot.displace(0, 6, 0);

        runToCompletion(15);

        assertTrue(follower.isAtTarget(),
                "failed to recover; ended " + follower.getPositionError() + "\" off");
        assertEquals(0.0, robot.getTruePose().getY(), 1.5);
    }

    @Test
    void correctsAgainstASteadySidewaysDisturbance() {
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0))));
        robot.setDisturbance(new ChassisSpeeds(0, 2.0, 0)); // 2"/s constant push

        double worstCrossTrack = 0;
        while (follower.isBusy()) {
            follower.update();
            robot.step(DT);
            worstCrossTrack = Math.max(worstCrossTrack, Math.abs(robot.getTruePose().getY()));
        }

        // A pure P controller cannot null a constant disturbance entirely -- it
        // settles at whatever offset makes kP * error equal the push. What it
        // must do is bound the error rather than let it grow without limit.
        assertTrue(worstCrossTrack < 4.0,
                "cross-track error grew to " + worstCrossTrack + "\"");
    }

    @Test
    void motorPowersNeverExceedFullScale() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierCurve(new Point(0, 0), new Point(30, 0), new Point(30, 30)))
                .setLinearHeadingInterpolation(0, Math.PI)
                .build();
        follower.followPath(chain);

        while (follower.isBusy()) {
            follower.update();
            for (double p : robot.getLastPowers()) {
                assertTrue(Math.abs(p) <= 1.0 + 1e-9, "power out of range: " + p);
                assertFalse(Double.isNaN(p), "NaN power");
            }
            robot.step(DT);
        }
    }

    @Test
    void breakFollowingStopsImmediatelyAndCutsPower() {
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0))));
        for (int i = 0; i < 20; i++) {
            follower.update();
            robot.step(DT);
        }

        follower.breakFollowing();
        assertFalse(follower.isBusy());
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-9);
        }
    }

    @Test
    void givesUpAfterTheSettleTimeoutRatherThanHangingForever() {
        // A robot that only delivers 10% of commanded speed cannot finish.
        // The follower must still terminate, and must report the miss.
        robot.setPowerScale(0.1);
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0))));

        double elapsed = runToCompletion(30);

        assertFalse(follower.isBusy(), "follower hung instead of timing out");
        assertFalse(follower.isAtTarget(), "should report that it did not arrive");
        assertTrue(elapsed < 20, "took " + elapsed + "s to give up");
    }

    @Test
    void startingANewPathClearsStateFromThePreviousOne() {
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(24, 0))));
        runToCompletion(10);

        Pose2d afterFirst = robot.getTruePose();
        // Hold the heading the robot already has, so that the only thing this
        // test can see on the first loop is leftover controller state -- not a
        // legitimate command to turn onto a new tangent.
        follower.followPath(new PathBuilder()
                .addPath(new BezierLine(
                        new Point(afterFirst.getX(), afterFirst.getY()),
                        new Point(afterFirst.getX(), afterFirst.getY() + 24)))
                .setConstantHeadingInterpolation(afterFirst.getHeading())
                .build());

        // The first loop of the new path must not lurch: the profile starts at
        // zero velocity, the robot is already at the start point, and the
        // controllers were reset -- so there is nothing left to command.
        follower.update();
        for (double p : robot.getLastPowers()) {
            assertTrue(Math.abs(p) < 0.1, "lurched on the first loop with power " + p);
        }

        runToCompletion(10);
        assertTrue(follower.isAtTarget());
        assertEquals(afterFirst.getY() + 24, robot.getTruePose().getY(), 1.5);
    }

    @Test
    void updateBeforeAnyPathIsHarmless() {
        follower.update();
        follower.update();
        assertFalse(follower.isBusy());
        assertEquals(0.0, follower.getPose().position.distanceTo(Vector2d.ZERO), 1e-9);
    }

    @Test
    void constantsGivenInCentimetresBehaveIdenticallyToInches() {
        FollowerConstants metric = FollowerConstants.builder(DistanceUnit.CM)
                .maxVelocity(76.2)        // 30 in/s
                .maxAcceleration(101.6)   // 40 in/s^2
                .kV(1.0 / MAX_WHEEL_VELOCITY)
                .translationalPID(2.0, 0.0, 0.0)
                .headingPID(3.0, 0.0, 0.0)
                .positionTolerance(2.54)  // 1 inch
                .headingTolerance(Math.toRadians(3))
                .build();

        assertEquals(30.0, metric.getMaxVelocity(), 1e-9);
        assertEquals(40.0, metric.getMaxAcceleration(), 1e-9);
        assertEquals(1.0, metric.getPositionTolerance(), 1e-9);

        PathFollower metricFollower = new PathFollower(robot, robot, metric, clock);
        metricFollower.followPath(PathChain.of(new BezierLine(
                new Point(0, 0), Point.of(60.96, 0, DistanceUnit.CM)))); // 24 inches

        while (metricFollower.isBusy()) {
            metricFollower.update();
            robot.step(DT);
        }
        assertEquals(24.0, robot.getTruePose().getX(), 1.0);
    }
}
