package com.platapus.pathing;

import com.platapus.pathing.control.MotionProfile;
import com.platapus.pathing.control.MotionState;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MotionProfileTest {

    private static final double EPS = 1e-9;

    /** 100" at 20"/s with 10"/s^2 both ways: 2s up, 3s cruise, 2s down. */
    private final MotionProfile trapezoid = new MotionProfile(100, 20, 10, 10);

    @Test
    void trapezoidHasTheExpectedPhaseDurations() {
        assertFalse(trapezoid.isTriangular());
        assertEquals(7.0, trapezoid.duration(), EPS);
        assertEquals(20.0, trapezoid.peakVelocity(), EPS);
    }

    @Test
    void accelerationPhaseFollowsHalfATSquared() {
        MotionState s = trapezoid.get(1.0);
        assertEquals(5.0, s.position, EPS);
        assertEquals(10.0, s.velocity, EPS);
        assertEquals(10.0, s.acceleration, EPS);
    }

    @Test
    void cruisePhaseHoldsVelocityWithNoAcceleration() {
        MotionState s = trapezoid.get(3.5);
        assertEquals(20.0 + 20.0 * 1.5, s.position, EPS);
        assertEquals(20.0, s.velocity, EPS);
        assertEquals(0.0, s.acceleration, EPS);
    }

    @Test
    void decelerationPhaseBrakesToRestExactlyAtTheEnd() {
        MotionState s = trapezoid.get(6.0);
        assertEquals(10.0, s.velocity, EPS);
        assertEquals(-10.0, s.acceleration, EPS);

        MotionState end = trapezoid.get(7.0);
        assertEquals(100.0, end.position, EPS);
        assertEquals(0.0, end.velocity, EPS);
    }

    @Test
    void positionIsMonotonicAndVelocityNeverNegative() {
        double previous = -1;
        for (double t = 0; t <= trapezoid.duration() + 1.0; t += 0.01) {
            MotionState s = trapezoid.get(t);
            assertTrue(s.position >= previous - 1e-9, "position went backwards at t=" + t);
            assertTrue(s.velocity >= -1e-9, "negative velocity at t=" + t);
            assertTrue(s.velocity <= 20.0 + 1e-9, "exceeded cruise velocity at t=" + t);
            previous = s.position;
        }
    }

    @Test
    void integratingVelocityReproducesPosition() {
        double dt = 1e-4;
        double integrated = 0.0;
        for (double t = 0; t < trapezoid.duration(); t += dt) {
            integrated += trapezoid.get(t).velocity * dt;
        }
        assertEquals(trapezoid.distance(), integrated, 0.01);
    }

    @Test
    void stateBeforeStartAndAfterEndIsClamped() {
        assertEquals(0.0, trapezoid.get(-5).position, EPS);
        assertEquals(0.0, trapezoid.get(-5).velocity, EPS);
        assertEquals(100.0, trapezoid.get(50).position, EPS);
        assertEquals(0.0, trapezoid.get(50).velocity, EPS);
    }

    @Test
    void shortPathDegeneratesToATriangle() {
        // 10" is too short to reach 20"/s: peak = sqrt(2*10*10*10/20) = 10"/s.
        MotionProfile triangle = new MotionProfile(10, 20, 10, 10);
        assertTrue(triangle.isTriangular());
        assertEquals(10.0, triangle.peakVelocity(), EPS);
        assertEquals(2.0, triangle.duration(), EPS);
        assertEquals(5.0, triangle.get(1.0).position, EPS);
        assertEquals(10.0, triangle.get(1.0).velocity, EPS);
        assertEquals(10.0, triangle.get(2.0).position, EPS);
    }

    @Test
    void asymmetricBrakingShortensTheProfile() {
        MotionProfile symmetric = new MotionProfile(100, 20, 10, 10);
        MotionProfile hardBraking = new MotionProfile(100, 20, 10, 40);
        assertTrue(hardBraking.duration() < symmetric.duration(),
                "braking harder should finish sooner");
        assertEquals(100.0, hardBraking.get(hardBraking.duration()).position, 1e-9);
    }

    @Test
    void zeroLengthProfileIsInstantaneous() {
        MotionProfile empty = new MotionProfile(0, 20, 10, 10);
        assertEquals(0.0, empty.duration(), EPS);
        assertEquals(0.0, empty.get(0).position, EPS);
        assertEquals(0.0, empty.get(1).velocity, EPS);
    }

    @Test
    void invalidLimitsAreRejected() {
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(-1, 20, 10, 10));
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(10, 0, 10, 10));
        assertThrows(IllegalArgumentException.class, () -> new MotionProfile(10, 20, -1, 10));
    }
}
