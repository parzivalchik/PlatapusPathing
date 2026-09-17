package com.platapus.pathing;

import com.platapus.pathing.control.ConstrainedProfile;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Curvature-limited speed: the profile must not plan a corner faster than the
 * robot could physically hold.
 */
class CorneringLimitTest {

    private static final double INF = Double.POSITIVE_INFINITY;

    // --- the formula ---------------------------------------------------------

    @Test
    void aStraightLineIsNeverLimited() {
        assertEquals(INF, ConstrainedProfile.lateralAccelerationLimit(0.0, 200));
        assertEquals(INF, ConstrainedProfile.lateralAccelerationLimit(1e-12, 200));
    }

    @Test
    void theLimitIsSqrtOfAccelerationOverCurvature() {
        // A 20-inch-radius corner (curvature 0.05) at 200 in/s^2 of grip:
        // v = sqrt(200 / 0.05) = 63.2 in/s.
        assertEquals(Math.sqrt(200 / 0.05),
                ConstrainedProfile.lateralAccelerationLimit(0.05, 200), 1e-9);

        // Equivalently: v^2 / r must come back out as the acceleration.
        double radius = 20, v = ConstrainedProfile.lateralAccelerationLimit(1 / radius, 200);
        assertEquals(200.0, v * v / radius, 1e-9);
    }

    @Test
    void tighterCornersAreSlower() {
        double gentle = ConstrainedProfile.lateralAccelerationLimit(0.01, 200);
        double sharp = ConstrainedProfile.lateralAccelerationLimit(0.2, 200);
        assertTrue(sharp < gentle, sharp + " should be under " + gentle);
    }

    @Test
    void theSignOfCurvatureDoesNotMatter() {
        assertEquals(ConstrainedProfile.lateralAccelerationLimit(0.08, 200),
                ConstrainedProfile.lateralAccelerationLimit(-0.08, 200), 1e-12);
    }

    @Test
    void anInfiniteOrAbsentLimitDisablesIt() {
        assertEquals(INF, ConstrainedProfile.lateralAccelerationLimit(0.2, INF));
        assertEquals(INF, ConstrainedProfile.lateralAccelerationLimit(0.2, 0));
        assertEquals(INF, ConstrainedProfile.lateralAccelerationLimit(0.2, -5));
    }

    // --- through the chain ---------------------------------------------------

    /** A hairpin: the tightest thing a three-point Bezier can make here. */
    private PathChain hairpin() {
        return PathChain.of(new BezierCurve(
                new Point(0, 0), new Point(30, 0), new Point(0, 12)));
    }

    /**
     * The same hairpin, but held at a fixed heading. Tangent heading round a
     * hairpin demands a near-180-degree spin, which makes heading rather than
     * cornering the thing under test.
     */
    private PathChain hairpinHoldingHeading() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(0, 0), new Point(30, 0), new Point(0, 12)))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    @Test
    void theChainReportsCurvatureAlongItself() {
        PathChain straight = PathChain.of(new BezierLine(new Point(0, 0), new Point(24, 0)));
        assertEquals(0.0, straight.curvatureAtArcLength(12), 1e-9);

        assertTrue(Math.abs(hairpin().curvatureAtArcLength(hairpin().length() / 2)) > 0.01,
                "a hairpin should have real curvature at its middle");
    }

    @Test
    void theSpeedLimitFallsWhereTheCornerIsTight() {
        PathChain chain = hairpin();
        double mid = chain.length() / 2;

        double unlimited = chain.speedLimitAtArcLength(mid, 60, INF);
        double limited = chain.speedLimitAtArcLength(mid, 60, 200);

        assertEquals(60.0, unlimited, 1e-9, "with no lateral limit, only the global cap applies");
        assertTrue(limited < 60.0,
                "the corner should bind before the global cap, got " + limited);
    }

    @Test
    void theCurvatureLimitNeverRaisesTheConfiguredCap() {
        // A gentle curve where the physics allows far more than the robot has.
        PathChain gentle = PathChain.of(new BezierCurve(
                new Point(0, 0), new Point(60, 0), new Point(120, 30)));
        assertEquals(30.0, gentle.speedLimitAtArcLength(gentle.length() / 2, 30, 400), 1e-6);
    }

    @Test
    void aSegmentCapAndTheCornerLimitBothApply() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierCurve(new Point(0, 0), new Point(30, 0), new Point(0, 12)))
                .setMaxVelocity(50)
                .build();

        double mid = chain.length() / 2;
        double both = chain.speedLimitAtArcLength(mid, 60, 200);
        double capOnly = chain.maxVelocityAtArcLength(mid, 60);

        assertEquals(50.0, capOnly, 1e-9);
        assertTrue(both <= capOnly, "whichever is lower must win, got " + both);
    }

    @Test
    void theTwoArgumentLimitStillIgnoresCurvature() {
        // Kept for callers that only care about configured caps; a change here
        // would silently alter what a segment cap means.
        PathChain chain = hairpin();
        assertEquals(60.0, chain.maxVelocityAtArcLength(chain.length() / 2, 60), 1e-9);
    }

    // --- through the profile and the follower --------------------------------

    @Test
    void aProfileOverAHairpinIsSlowerWithTheLimitOn() {
        PathChain chain = hairpin();

        ConstrainedProfile off = new ConstrainedProfile(chain.length(),
                s -> chain.speedLimitAtArcLength(s, 60, INF), 50, 60);
        ConstrainedProfile on = new ConstrainedProfile(chain.length(),
                s -> chain.speedLimitAtArcLength(s, 60, 200), 50, 60);

        assertTrue(on.duration() > off.duration(),
                "cornering slowly costs time: " + off.duration() + "s -> " + on.duration() + "s");
        assertTrue(on.peakVelocity() < off.peakVelocity());
    }

    @Test
    void theProfileBrakesBeforeTheCornerNotAtIt() {
        // The property that makes this usable: arriving already slow, rather
        // than discovering the corner at speed and needing impossible braking.
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(-60, 0), new Point(0, 0)))
                .addPath(new BezierCurve(new Point(0, 0), new Point(30, 0), new Point(0, 12)))
                .build();

        double lateral = 150;
        ConstrainedProfile profile = new ConstrainedProfile(chain.length(),
                s -> chain.speedLimitAtArcLength(s, 60, lateral), 50, 60);

        double cornerStart = 60.0;                      // where the straight ends

        // Sample the peak rather than a fixed point: early on the straight the
        // robot is still winding up from rest, so a low reading there would say
        // nothing about braking.
        double peakOnStraight = 0;
        for (double s = 0; s <= cornerStart; s += 0.5) {
            peakOnStraight = Math.max(peakOnStraight, profile.velocityAtArcLength(s));
        }

        double atEntry = profile.velocityAtArcLength(cornerStart);
        assertTrue(atEntry < peakOnStraight * 0.9,
                "should arrive at the corner well below cruise: " + atEntry
                        + " against a peak of " + peakOnStraight);

        // And the approach ramps down rather than stepping: both of these are
        // past the acceleration ramp, so the comparison is meaningful.
        assertTrue(profile.velocityAtArcLength(58) < profile.velocityAtArcLength(45),
                "the last stretch of the straight should already be slowing");

        // And nowhere on the path does the planned speed exceed what the corner
        // there physically allows.
        for (double s = 0; s <= chain.length(); s += 0.5) {
            double allowed = chain.speedLimitAtArcLength(s, 60, lateral);
            assertTrue(profile.velocityAtArcLength(s) <= allowed + 0.6,
                    "planned " + profile.velocityAtArcLength(s)
                            + " but only " + allowed + " allowed at s=" + s);
        }
    }

    @Test
    void aStraightPathIsUnaffectedByEnablingTheLimit() {
        // Turning this on must not quietly slow down paths that do not corner.
        PathChain straight = PathChain.of(new BezierLine(new Point(0, 0), new Point(72, 0)));

        ConstrainedProfile off = new ConstrainedProfile(straight.length(),
                s -> straight.speedLimitAtArcLength(s, 40, INF), 50, 60);
        ConstrainedProfile on = new ConstrainedProfile(straight.length(),
                s -> straight.speedLimitAtArcLength(s, 40, 150), 50, 60);

        assertEquals(off.duration(), on.duration(), 1e-9);
    }

    @Test
    void itIsOffByDefaultSoExistingTuningIsUnchanged() {
        FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(40).maxAcceleration(50).kV(0.017)
                .build();
        assertEquals(INF, constants.getMaxLateralAcceleration());
    }

    @Test
    void theLimitIsUnitAware() {
        FollowerConstants metric = FollowerConstants.builder(DistanceUnit.CM)
                .maxVelocity(100).maxAcceleration(120).kV(0.017)
                .maxLateralAcceleration(508)   // 200 in/s^2
                .build();
        assertEquals(200.0, metric.getMaxLateralAcceleration(), 1e-9);
    }

    @Test
    void aNonPositiveLimitIsRejected() {
        assertThrows(IllegalStateException.class,
                () -> FollowerConstants.builder(DistanceUnit.INCH)
                        .maxVelocity(40).maxAcceleration(50).kV(0.017)
                        .maxLateralAcceleration(0).build());
    }

    @Test
    void theFollowerActuallyDrivesTheCornerSlower() {
        double dt = 0.02;
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();

        PathChain chain = hairpinHoldingHeading();

        double[] durations = new double[2];
        for (int i = 0; i < 2; i++) {
            FollowerConstants.Builder b = FollowerConstants.builder(DistanceUnit.INCH)
                    .maxVelocity(50).maxAcceleration(50).maxDeceleration(60)
                    .kV(1.0 / 60)
                    .translationalPID(2.0, 0.0, 0.0)
                    .headingPID(3.0, 0.0, 0.0)
                    .positionTolerance(1.5)
                    .headingTolerance(Math.toRadians(5));
            if (i == 1) {
                b.maxLateralAcceleration(150);
            }

            SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
            SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);
            PathFollower follower = new PathFollower(robot, robot, b.build(), clock);

            follower.followPath(chain);
            durations[i] = follower.getProfile().duration();

            int loops = 0;
            while (follower.isBusy() && loops++ < 3000) {
                follower.update();
                robot.step(dt);
            }

            // The point is that the corner is drivable either way, so check the
            // robot actually got to the end rather than that every tolerance
            // was met.
            assertEquals(chain.endState().point.x, robot.getTruePose().getX(), 2.0,
                    "run " + i + " x");
            assertEquals(chain.endState().point.y, robot.getTruePose().getY(), 2.0,
                    "run " + i + " y");
        }

        assertTrue(durations[1] > durations[0],
                "the cornering limit should lengthen the run: "
                        + durations[0] + "s -> " + durations[1] + "s");
    }

    @Test
    void anImpossiblyTightCornerStillProducesAFiniteProfile() {
        // A cusp has enormous curvature. The limit must not stall the profile.
        PathChain cusp = PathChain.of(new BezierCurve(
                new Point(0, 0), new Point(20, 0), new Point(0, 0.01)));

        ConstrainedProfile profile = new ConstrainedProfile(cusp.length(),
                s -> cusp.speedLimitAtArcLength(s, 40, 100), 50, 60);

        assertTrue(Double.isFinite(profile.duration()), "duration diverged");
        assertFalse(profile.duration() > 600, "took " + profile.duration() + "s");
        assertEquals(cusp.length(), profile.get(profile.duration()).position, 1e-6);
    }
}
