package com.platapus.pathing;

import com.platapus.pathing.control.ConstrainedProfile;
import com.platapus.pathing.control.MotionProfile;
import com.platapus.pathing.control.MotionState;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ConstrainedProfileTest {

    @Test
    void withAConstantCeilingItReproducesTheAnalyticTrapezoid() {
        // The numerical solver has to agree with the closed form, or every
        // existing tuning assumption quietly shifts.
        MotionProfile analytic = new MotionProfile(100, 20, 10, 10);
        ConstrainedProfile numeric = new ConstrainedProfile(100, 20, 10, 10);

        assertEquals(analytic.duration(), numeric.duration(), 0.02);
        assertEquals(analytic.peakVelocity(), numeric.peakVelocity(), 0.1);

        for (double t = 0; t <= analytic.duration(); t += 0.05) {
            assertEquals(analytic.get(t).position, numeric.get(t).position, 0.3,
                    "position at t=" + t);
            assertEquals(analytic.get(t).velocity, numeric.get(t).velocity, 0.3,
                    "velocity at t=" + t);
        }
    }

    @Test
    void aTriangularCaseAlsoMatchesTheAnalyticForm() {
        MotionProfile analytic = new MotionProfile(10, 20, 10, 10);
        ConstrainedProfile numeric = new ConstrainedProfile(10, 20, 10, 10);

        assertEquals(analytic.duration(), numeric.duration(), 0.02);
        assertEquals(analytic.peakVelocity(), numeric.peakVelocity(), 0.2);
    }

    @Test
    void startsAndEndsAtRest() {
        ConstrainedProfile profile = new ConstrainedProfile(60, 30, 40, 50);
        assertEquals(0.0, profile.get(0).velocity, 1e-9);
        assertEquals(0.0, profile.get(profile.duration()).velocity, 1e-9);
        assertEquals(60.0, profile.get(profile.duration()).position, 1e-6);
    }

    @Test
    void positionIsMonotonicAndVelocityNeverNegative() {
        ConstrainedProfile profile = new ConstrainedProfile(
                80, s -> (s > 30 && s < 50) ? 6.0 : 40.0, 30, 40);

        double previous = -1;
        for (double t = 0; t <= profile.duration() + 0.5; t += 0.01) {
            MotionState state = profile.get(t);
            assertTrue(state.position >= previous - 1e-9, "went backwards at t=" + t);
            assertTrue(state.velocity >= -1e-9, "negative velocity at t=" + t);
            previous = state.position;
        }
    }

    @Test
    void aSlowZoneIsRespectedWhileItIsInForce() {
        ConstrainedProfile profile = new ConstrainedProfile(
                80, s -> (s > 30 && s < 50) ? 6.0 : 40.0, 30, 40);

        for (double s = 32; s <= 48; s += 0.5) {
            assertTrue(profile.velocityAtArcLength(s) <= 6.0 + 1e-6,
                    "exceeded the slow-zone cap at " + s + "\": "
                            + profile.velocityAtArcLength(s));
        }
    }

    @Test
    void theProfileBrakesBeforeReachingASlowZoneNotAtIt() {
        // The thing a naive per-segment profile gets wrong: arriving at the slow
        // section still going fast and then needing impossible braking.
        double decel = 40;
        ConstrainedProfile profile = new ConstrainedProfile(
                80, s -> (s > 30 && s < 50) ? 6.0 : 40.0, 30, decel);

        // Entering the zone at 6"/s from 40"/s needs (40^2 - 6^2)/(2*40) = 19.6"
        // of runway, so the robot must already be slowing well before s=30 --
        // and must be down to the cap as soon as it is actually inside.
        double atCruise = profile.velocityAtArcLength(15);
        assertTrue(profile.velocityAtArcLength(25) < atCruise,
                "should already be braking 5\" out, was "
                        + profile.velocityAtArcLength(25) + " against a cruise of "
                        + atCruise);
        assertTrue(profile.velocityAtArcLength(31) <= 6.0 + 1e-6,
                "should be at the cap once inside the zone, was "
                        + profile.velocityAtArcLength(31));

        // Braking is continuous into the zone rather than a cliff at the edge.
        assertTrue(profile.velocityAtArcLength(25) > profile.velocityAtArcLength(29),
                "the approach should ramp down, not step");
    }

    @Test
    void deceleratingIntoTheZoneNeverExceedsTheBrakingLimit() {
        double accel = 30;
        double decel = 40;
        ConstrainedProfile profile = new ConstrainedProfile(
                80, s -> (s > 30 && s < 50) ? 6.0 : 40.0, accel, decel);

        double previous = profile.get(0).velocity;
        double dt = 0.005;
        for (double t = dt; t <= profile.duration(); t += dt) {
            double v = profile.get(t).velocity;
            double rate = (v - previous) / dt;
            assertTrue(rate <= accel + 2.0, "accelerated too hard at t=" + t + ": " + rate);
            assertTrue(rate >= -decel - 2.0, "braked too hard at t=" + t + ": " + rate);
            previous = v;
        }
    }

    @Test
    void aSlowZoneMakesTheRunTakeLonger() {
        ConstrainedProfile unconstrained = new ConstrainedProfile(80, 40, 30, 40);
        ConstrainedProfile constrained = new ConstrainedProfile(
                80, s -> (s > 30 && s < 50) ? 6.0 : 40.0, 30, 40);

        assertTrue(constrained.duration() > unconstrained.duration() * 1.5,
                "a 20\" crawl should cost real time: " + unconstrained.duration()
                        + "s -> " + constrained.duration() + "s");
    }

    @Test
    void integratingVelocityReproducesTheDistance() {
        ConstrainedProfile profile = new ConstrainedProfile(
                80, s -> (s > 30 && s < 50) ? 6.0 : 40.0, 30, 40);

        double dt = 1e-4;
        double integrated = 0;
        for (double t = 0; t < profile.duration(); t += dt) {
            integrated += profile.get(t).velocity * dt;
        }
        assertEquals(80.0, integrated, 0.2);
    }

    @Test
    void zeroLengthProfileIsInstantaneous() {
        ConstrainedProfile profile = new ConstrainedProfile(0, 30, 40, 40);
        assertEquals(0.0, profile.duration(), 1e-9);
        assertEquals(0.0, profile.get(1).velocity, 1e-9);
    }

    @Test
    void aNonsenseCeilingDoesNotHangTheProfile() {
        // A zero or negative limit would otherwise mean "never arrive".
        ConstrainedProfile profile = new ConstrainedProfile(20, s -> 0.0, 30, 40);
        assertTrue(Double.isFinite(profile.duration()));
        assertEquals(20.0, profile.get(profile.duration()).position, 1e-6);
    }

    @Test
    void invalidLimitsAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new ConstrainedProfile(-1, 20, 10, 10));
        assertThrows(IllegalArgumentException.class,
                () -> new ConstrainedProfile(10, 20, 0, 10));
        assertThrows(IllegalArgumentException.class,
                () -> new ConstrainedProfile(10, null, 10, 10));
    }

    // --- integration with the path layer --------------------------------------

    @Test
    void perSegmentCapsFlowThroughTheChain() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
                .addPath(new BezierLine(new Point(24, 0), new Point(48, 0)))
                .setMaxVelocity(8)
                .build();

        assertTrue(chain.hasVelocityOverrides());
        assertEquals(40.0, chain.maxVelocityAtArcLength(10, 40), 1e-9,
                "first segment has no cap");
        assertEquals(8.0, chain.maxVelocityAtArcLength(36, 40), 1e-9,
                "second segment is capped");
    }

    @Test
    void aSegmentCapNeverRaisesTheGlobalLimit() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
                .setMaxVelocity(100)
                .build();

        assertEquals(30.0, chain.maxVelocityAtArcLength(12, 30), 1e-9,
                "a segment must not be allowed to exceed the robot's own limit");
    }

    @Test
    void chainsWithNoCapsReportNoOverrides() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
                .build();
        assertFalse(chain.hasVelocityOverrides());
        assertEquals(40.0, chain.maxVelocityAtArcLength(12, 40), 1e-9);
    }

    @Test
    void capsAreUnitAware() {
        PathChain metric = new PathBuilder()
                .setUnit(DistanceUnit.CM)
                .line(0, 0, 60.96, 0)     // 24 inches
                .setMaxVelocity(25.4)      // 10 in/s
                .build();

        assertEquals(10.0, metric.maxVelocityAtArcLength(5, 40), 1e-9);
    }

    @Test
    void aCappedSegmentSlowsTheProfileBuiltFromTheChain() {
        PathChain open = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(48, 0)))
                .addPath(new BezierLine(new Point(48, 0), new Point(96, 0)))
                .build();

        PathChain careful = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(48, 0)))
                .addPath(new BezierLine(new Point(48, 0), new Point(96, 0)))
                .setMaxVelocity(5)
                .build();

        ConstrainedProfile fast = new ConstrainedProfile(
                open.length(), s -> open.maxVelocityAtArcLength(s, 40), 30, 40);
        ConstrainedProfile slow = new ConstrainedProfile(
                careful.length(), s -> careful.maxVelocityAtArcLength(s, 40), 30, 40);

        assertTrue(slow.duration() > fast.duration() * 2,
                "capping the second half should roughly dominate the run time: "
                        + fast.duration() + "s vs " + slow.duration() + "s");
        assertTrue(slow.velocityAtArcLength(70) <= 5.0 + 1e-6);
    }

    @Test
    void builderRejectsACapBeforeAnyPath() {
        assertThrows(IllegalStateException.class,
                () -> new PathBuilder().setMaxVelocity(10));
    }

    @Test
    void aNegativeCapIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> new PathBuilder()
                        .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                        .setMaxVelocity(-5));
    }
}
