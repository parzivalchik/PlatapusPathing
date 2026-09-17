package com.platapus.pathing;

import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.PathState;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PathChainTest {

    private static final double EPS = 1e-9;
    private static final double ARC_EPS = 1e-3;

    /** Two straight 10" segments forming an L: right, then up. */
    private PathChain lShape() {
        return new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .addPath(new BezierLine(new Point(10, 0), new Point(10, 10)))
                .build();
    }

    @Test
    void chainLengthIsTheSumOfItsSegments() {
        PathChain chain = lShape();
        assertEquals(2, chain.size());
        assertEquals(20.0, chain.length(), ARC_EPS);
        assertEquals(0.0, chain.segmentStartArcLength(0), EPS);
        assertEquals(10.0, chain.segmentStartArcLength(1), ARC_EPS);
    }

    @Test
    void arcLengthAddressingCrossesSegmentBoundariesSeamlessly() {
        PathChain chain = lShape();

        PathState first = chain.stateAtArcLength(5);
        assertEquals(0, first.segmentIndex);
        assertEquals(5.0, first.point.x, ARC_EPS);
        assertEquals(0.0, first.point.y, ARC_EPS);

        PathState second = chain.stateAtArcLength(15);
        assertEquals(1, second.segmentIndex);
        assertEquals(10.0, second.point.x, ARC_EPS);
        assertEquals(5.0, second.point.y, ARC_EPS);
    }

    @Test
    void arcLengthIsClampedToTheChain() {
        PathChain chain = lShape();
        assertEquals(0.0, chain.stateAtArcLength(-10).arcLength, EPS);
        assertEquals(chain.length(), chain.stateAtArcLength(1000).arcLength, ARC_EPS);
        assertEquals(10.0, chain.stateAtArcLength(1000).point.y, ARC_EPS);
    }

    @Test
    void steppingArcLengthProducesEvenlySpacedPointsAcrossTheWholeChain() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .addPath(new BezierCurve(new Point(10, 0), new Point(20, 0), new Point(20, 10)))
                .build();

        int steps = 40;
        double step = chain.length() / steps;
        for (int i = 0; i < steps; i++) {
            Vector2d a = chain.stateAtArcLength(i * step).point;
            Vector2d b = chain.stateAtArcLength((i + 1) * step).point;
            assertEquals(step, a.distanceTo(b), 0.05, "uneven spacing at step " + i);
        }
    }

    @Test
    void startAndEndStatesAreTheChainEndpoints() {
        PathChain chain = lShape();
        assertEquals(0.0, chain.startState().point.distanceTo(new Vector2d(0, 0)), EPS);
        assertEquals(0.0, chain.endState().point.distanceTo(new Vector2d(10, 10)), EPS);
    }

    @Test
    void defaultHeadingFollowsTheTangent() {
        PathChain chain = lShape();
        // First segment runs along +x, second along +y.
        assertEquals(0.0, chain.stateAtArcLength(5).heading, EPS);
        assertEquals(Math.PI / 2, chain.stateAtArcLength(15).heading, EPS);
    }

    @Test
    void constantHeadingInterpolationHoldsOneAngle() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        for (double s = 0; s <= 10; s += 1.0) {
            assertEquals(Math.PI / 2, chain.stateAtArcLength(s).heading, 1e-9);
        }
    }

    @Test
    void linearHeadingInterpolationSweepsAcrossTheSegment() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                .build();
        assertEquals(0.0, chain.stateAtSegment(0, 0).heading, 1e-9);
        assertEquals(Math.toRadians(45), chain.stateAtSegment(0, 0.5).heading, 1e-9);
        assertEquals(Math.toRadians(90), chain.stateAtSegment(0, 1).heading, 1e-9);
    }

    @Test
    void linearHeadingInterpolationTakesTheShortWayAroundTheWrap() {
        // 350 deg -> 10 deg is a 20 degree turn forwards, not 340 backwards.
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(10))
                .build();
        double midpoint = chain.stateAtSegment(0, 0.5).heading;
        assertEquals(0.0, Math.abs(midpoint), 1e-9, "should pass through 0/360, got " + midpoint);
    }

    @Test
    void reversedHeadingInterpolationTakesTheLongWayAround() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .setReversedHeadingInterpolation(Math.toRadians(350), Math.toRadians(10))
                .build();
        // Going the long way, the midpoint is on the far side, near 180.
        assertEquals(Math.PI, Math.abs(chain.stateAtSegment(0, 0.5).heading), 1e-9);
    }

    @Test
    void reverseTangentFacesBackwardsAlongTravel() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .setReverseTangentHeadingInterpolation()
                .build();
        assertEquals(Math.PI, Math.abs(chain.stateAtArcLength(5).heading), 1e-9);
    }

    @Test
    void headingInterpolationAppliesOnlyToTheMostRecentlyAddedPath() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(10, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(new BezierLine(new Point(10, 0), new Point(20, 0)))
                .build();
        assertEquals(Math.PI / 2, chain.stateAtArcLength(5).heading, 1e-9);
        // Second segment never had a heading set, so it defaults to tangent (+x).
        assertEquals(0.0, chain.stateAtArcLength(15).heading, 1e-9);
    }

    @Test
    void projectionFindsTheNearestPointAndAdvancesWithTheRobot() {
        PathChain chain = lShape();

        PathState early = chain.project(new Vector2d(3, 1), 0, 0.0);
        assertEquals(0, early.segmentIndex);
        assertEquals(3.0, early.point.x, 1e-3);

        PathState late = chain.project(new Vector2d(11, 6), 1, 0.5);
        assertEquals(1, late.segmentIndex);
        assertEquals(6.0, late.point.y, 1e-3);
    }

    @Test
    void projectionLooksAheadOneSegmentSoItCanCrossABoundary() {
        PathChain chain = lShape();
        // Robot is past the corner but the follower still thinks it is on
        // segment 0; the one-segment lookahead should catch it up.
        PathState state = chain.project(new Vector2d(10, 4), 0, 0.99);
        assertEquals(1, state.segmentIndex);
        assertEquals(4.0, state.point.y, 1e-3);
    }

    @Test
    void projectionDoesNotJumpBackwardsOnASelfCrossingChain() {
        // A chain that returns near its own start. A robot at the end must not
        // be told it is back at the beginning.
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(0, 0), new Point(20, 0)))
                .addPath(new BezierLine(new Point(20, 0), new Point(20, 20)))
                .addPath(new BezierLine(new Point(20, 20), new Point(0.5, 20)))
                .addPath(new BezierLine(new Point(0.5, 20), new Point(0.5, 0.5)))
                .build();

        PathState state = chain.project(new Vector2d(0.5, 1.0), 3, 0.95);
        assertEquals(3, state.segmentIndex, "should stay on the final segment");
        assertTrue(state.arcLength > 50, "arc length should be near the end, was "
                + state.arcLength);
    }

    @Test
    void builderUnitAppliesToItsOwnCoordinateHelpers() {
        PathChain metric = new PathBuilder()
                .setUnit(DistanceUnit.CM)
                .line(0, 0, 25.4, 0)   // 10 inches
                .build();
        assertEquals(10.0, metric.length(), ARC_EPS);
        assertEquals(10.0, metric.endState().point.x, ARC_EPS);
    }

    @Test
    void pointsCarryTheirOwnUnitRegardlessOfBuilderUnit() {
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(
                        Point.of(0, 0, DistanceUnit.CM),
                        Point.of(25.4, 0, DistanceUnit.CM)))
                .build();
        assertEquals(10.0, chain.length(), ARC_EPS);
    }

    @Test
    void curveHelperBuildsAMultiPointBezier() {
        PathChain chain = new PathBuilder()
                .curve(0, 0, 10, 0, 10, 10)
                .build();
        assertEquals(1, chain.size());
        assertEquals(0.0, chain.endState().point.distanceTo(new Vector2d(10, 10)), EPS);
    }

    @Test
    void builderRejectsMisuse() {
        assertThrows(IllegalStateException.class, () -> new PathBuilder().build());
        assertThrows(IllegalStateException.class,
                () -> new PathBuilder().setConstantHeadingInterpolation(0));
        assertThrows(IllegalArgumentException.class,
                () -> new PathBuilder().curve(0, 0, 1, 1));
        assertThrows(IllegalArgumentException.class,
                () -> new PathChain(java.util.Collections.emptyList()));
    }

    @Test
    void addressingASegmentThatDoesNotExistFails() {
        PathChain chain = PathChain.of(new BezierLine(new Point(0, 0), new Point(1, 0)));
        assertThrows(IndexOutOfBoundsException.class, () -> chain.stateAtSegment(5, 0));
    }
}
