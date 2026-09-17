package com.platapus.pathing;

import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.Point;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BezierPathTest {

    private static final double EPS = 1e-9;
    /** Arc-length results come from a 200-sample table, so they are approximate. */
    private static final double ARC_EPS = 1e-3;

    @Nested
    @DisplayName("BezierLine")
    class Line {

        private final BezierLine line = new BezierLine(new Point(0, 0), new Point(10, 0));

        @Test
        void interpolatesLinearlyBetweenEndpoints() {
            assertEquals(0.0, line.getPoint(0).x, EPS);
            assertEquals(5.0, line.getPoint(0.5).x, EPS);
            assertEquals(10.0, line.getPoint(1).x, EPS);
            assertEquals(0.0, line.getPoint(0.5).y, EPS);
        }

        @Test
        void derivativeIsTheConstantEndToEndVector() {
            // For a degree-1 Bezier the derivative control point is 1 * (P1 - P0).
            for (double t = 0; t <= 1.0; t += 0.25) {
                assertEquals(10.0, line.getDerivative(t).x, EPS);
                assertEquals(0.0, line.getDerivative(t).y, EPS);
            }
        }

        @Test
        void secondDerivativeAndCurvatureVanish() {
            assertEquals(0.0, line.getSecondDerivative(0.5).norm(), EPS);
            assertEquals(0.0, line.getCurvature(0.5), EPS);
        }

        @Test
        void lengthIsTheStraightLineDistance() {
            assertEquals(10.0, line.length(), ARC_EPS);
        }

        @Test
        void arcLengthIsProportionalToParameterOnAStraightLine() {
            assertEquals(2.5, line.arcLengthAt(0.25), ARC_EPS);
            assertEquals(5.0, line.arcLengthAt(0.5), ARC_EPS);
        }

        @Test
        void tAtArcLengthInvertsArcLengthAt() {
            assertEquals(0.75, line.tAtArcLength(7.5), ARC_EPS);
        }

        @Test
        void parameterIsClampedOutsideTheUnitInterval() {
            assertEquals(0.0, line.getPoint(-3).x, EPS);
            assertEquals(10.0, line.getPoint(4).x, EPS);
        }

        @Test
        void closestTProjectsPerpendicularly() {
            // A point 5 units off to the side of the midpoint projects to t=0.5.
            assertEquals(0.5, line.getClosestT(new Vector2d(5, 5), 0.5), 1e-4);
        }

        @Test
        void closestTClampsToTheEndsForPointsBeyondThem() {
            assertEquals(0.0, line.getClosestT(new Vector2d(-20, 0), 0.0), 1e-4);
            assertEquals(1.0, line.getClosestT(new Vector2d(30, 0), 1.0), 1e-4);
        }
    }

    @Nested
    @DisplayName("Quadratic BezierCurve")
    class Quadratic {

        // P0 = (0,0), P1 = (10,0), P2 = (10,10) -- a quarter-turn to the left.
        private final BezierCurve curve = new BezierCurve(
                new Point(0, 0), new Point(10, 0), new Point(10, 10));

        @Test
        void passesThroughItsEndpointsButNotItsControlPoint() {
            assertEquals(0.0, curve.getPoint(0).distanceTo(new Vector2d(0, 0)), EPS);
            assertEquals(0.0, curve.getPoint(1).distanceTo(new Vector2d(10, 10)), EPS);
            assertTrue(curve.getPoint(0.5).distanceTo(new Vector2d(10, 0)) > 1.0,
                    "the curve should be pulled toward, not through, its control point");
        }

        @Test
        void midpointMatchesTheClosedForm() {
            // B(1/2) = 1/4 P0 + 1/2 P1 + 1/4 P2
            assertEquals(7.5, curve.getPoint(0.5).x, EPS);
            assertEquals(2.5, curve.getPoint(0.5).y, EPS);
        }

        @Test
        void derivativeMatchesTheClosedForm() {
            // B'(t) is the degree-1 Bezier over 2*(P1-P0) = (20,0) and 2*(P2-P1) = (0,20).
            assertEquals(20.0, curve.getDerivative(0).x, EPS);
            assertEquals(0.0, curve.getDerivative(0).y, EPS);

            assertEquals(10.0, curve.getDerivative(0.5).x, EPS);
            assertEquals(10.0, curve.getDerivative(0.5).y, EPS);

            assertEquals(0.0, curve.getDerivative(1).x, EPS);
            assertEquals(20.0, curve.getDerivative(1).y, EPS);
        }

        @Test
        void derivativeMatchesANumericalDifference() {
            double h = 1e-6;
            for (double t : new double[]{0.1, 0.35, 0.6, 0.9}) {
                Vector2d numerical = curve.getPoint(t + h).minus(curve.getPoint(t - h)).div(2 * h);
                Vector2d analytic = curve.getDerivative(t);
                assertEquals(numerical.x, analytic.x, 1e-4, "d/dt x at t=" + t);
                assertEquals(numerical.y, analytic.y, 1e-4, "d/dt y at t=" + t);
            }
        }

        @Test
        void secondDerivativeIsConstantForAQuadratic() {
            Vector2d expected = new Vector2d(-20, 20);
            for (double t = 0; t <= 1.0; t += 0.25) {
                assertEquals(expected.x, curve.getSecondDerivative(t).x, EPS);
                assertEquals(expected.y, curve.getSecondDerivative(t).y, EPS);
            }
        }

        @Test
        void unitTangentHasUnitLength() {
            for (double t = 0; t <= 1.0; t += 0.1) {
                assertEquals(1.0, curve.getUnitTangent(t).norm(), 1e-9);
            }
        }

        @Test
        void curvatureIsPositiveWhenTurningLeft() {
            // d1 = (10,10), d2 = (-20,20): cross = 400, |d1|^3 = 200^1.5
            double expected = 400.0 / Math.pow(Math.sqrt(200.0), 3);
            assertEquals(expected, curve.getCurvature(0.5), 1e-9);
            assertTrue(curve.getCurvature(0.5) > 0);
        }

        @Test
        void curvatureIsNegativeForTheMirroredCurveTurningRight() {
            BezierCurve mirrored = new BezierCurve(
                    new Point(0, 0), new Point(10, 0), new Point(10, -10));
            assertTrue(mirrored.getCurvature(0.5) < 0);
        }

        @Test
        void lengthLiesBetweenTheChordAndTheControlPolygon() {
            double chord = Math.hypot(10, 10);
            double polygon = 10 + 10;
            assertTrue(curve.length() > chord,
                    "a curve is longer than its chord: " + curve.length());
            assertTrue(curve.length() < polygon,
                    "a Bezier is shorter than its control polygon: " + curve.length());
        }

        @Test
        void arcLengthIsMonotonicAndSpansTheFullLength() {
            double previous = -1;
            for (double t = 0; t <= 1.0; t += 0.05) {
                double s = curve.arcLengthAt(t);
                assertTrue(s >= previous, "arc length must not decrease at t=" + t);
                previous = s;
            }
            assertEquals(0.0, curve.arcLengthAt(0), EPS);
            assertEquals(curve.length(), curve.arcLengthAt(1), ARC_EPS);
        }

        @Test
        void tAtArcLengthRoundTrips() {
            for (double t = 0.05; t < 1.0; t += 0.05) {
                double s = curve.arcLengthAt(t);
                assertEquals(t, curve.tAtArcLength(s), 1e-3, "round trip at t=" + t);
            }
        }

        @Test
        void arcLengthParameterisationIsEvenlySpacedInDistance() {
            // Stepping by equal arc length must produce equal distances on the
            // ground -- the whole reason the table exists. Stepping t directly
            // would not.
            int steps = 20;
            double step = curve.length() / steps;
            double expected = step;
            for (int i = 0; i < steps; i++) {
                Vector2d a = curve.getPoint(curve.tAtArcLength(i * step));
                Vector2d b = curve.getPoint(curve.tAtArcLength((i + 1) * step));
                assertEquals(expected, a.distanceTo(b), 0.05,
                        "uneven spacing in step " + i);
            }
        }

        @Test
        void closestTFindsTheNearestPointOnTheCurve() {
            Vector2d onCurve = curve.getPoint(0.3);
            assertEquals(0.3, curve.getClosestT(onCurve, 0.0), 1e-3);
        }
    }

    @Nested
    @DisplayName("Higher-degree and degenerate cases")
    class EdgeCases {

        @Test
        void cubicIsSupported() {
            BezierCurve cubic = new BezierCurve(
                    new Point(0, 0), new Point(0, 10), new Point(10, 10), new Point(10, 0));
            assertEquals(3, cubic.degree());
            // B(1/2) = 1/8 P0 + 3/8 P1 + 3/8 P2 + 1/8 P3
            assertEquals(1.25 + 3.75, cubic.getPoint(0.5).x, EPS);
            assertEquals(3.75 + 3.75, cubic.getPoint(0.5).y, EPS);
        }

        @Test
        void degenerateZeroLengthLineDoesNotProduceNaN() {
            BezierLine degenerate = new BezierLine(new Point(4, 4), new Point(4, 4));
            assertEquals(0.0, degenerate.length(), EPS);
            assertEquals(0.0, degenerate.getUnitTangent(0.5).norm(), EPS);
            assertEquals(0.0, degenerate.getCurvature(0.5), EPS);
            assertEquals(0.0, degenerate.tAtArcLength(1.0), EPS);
        }

        @Test
        void curveRejectsFewerThanThreePoints() {
            assertThrows(IllegalArgumentException.class,
                    () -> new BezierCurve(new Point(0, 0), new Point(1, 1)));
        }
    }
}
