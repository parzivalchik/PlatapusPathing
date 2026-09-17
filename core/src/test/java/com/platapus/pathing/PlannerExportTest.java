package com.platapus.pathing;

import com.platapus.pathing.control.ConstrainedProfile;
import com.platapus.pathing.geometry.FieldCoordinates;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Guards the contract between the alliance planner and this library.
 *
 * <p>The bodies below are pasted verbatim out of the planner's Java export. If
 * someone renames a builder method or changes an argument type, this stops
 * compiling -- which is the point. The planner has no compile-time dependency on
 * the library, so this test is the only thing that would otherwise notice the
 * export had gone stale.
 *
 * <p>The numbers also encode the planner's arithmetic: if its Bezier evaluation
 * or arc-length table ever drifts from the library's, the lengths asserted here
 * stop matching what the tool displayed.
 */
class PlannerExportTest {

    @Test
    void exportedRobot1PathCompilesAndIsInFieldCoordinates() {
        // --- pasted from the planner, unedited -----------------------------
        // Robot 1 -- field coordinates, inches.
        // Tell the localizer where the robot is placed:
        //   .startPose(new Pose2d(-60.000, -36.000, Math.toRadians(0.00)))
        PathChain robot1 = new PathBuilder()
                .addPath(new BezierLine(new Point(-60.000, -36.000),
                        new Point(-24.000, -36.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(new Point(-24.000, -36.000),
                        new Point(-6.000, -36.000), new Point(0.000, -12.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0.00), Math.toRadians(90.00))
                .build();
        // -------------------------------------------------------------------

        assertEquals(2, robot1.size());
        assertEquals(36.0, robot1.getSegment(0).length(), 1e-3);

        // The whole point of the frame change: these are absolute positions, so
        // the chain starts where the robot is placed rather than at (0, 0).
        assertEquals(-60.0, robot1.startState().point.x, 1e-3);
        assertEquals(-36.0, robot1.startState().point.y, 1e-3);
        assertEquals(0.0, robot1.endState().point.x, 1e-3);
        assertEquals(-12.0, robot1.endState().point.y, 1e-3);

        assertEquals(0.0, robot1.stateAtSegment(1, 0).heading, 1e-9);
        assertEquals(Math.PI / 2, robot1.stateAtSegment(1, 1).heading, 1e-9);
    }

    @Test
    void theExportedStartPoseIsTheChainsOwnFirstPoint() {
        // The planner derives the start position from the path, so a pasted
        // startPose and the chain's first point cannot disagree.
        Pose2d startPose = new Pose2d(-60.000, -36.000, Math.toRadians(0.00));

        PathChain robot1 = new PathBuilder()
                .addPath(new BezierLine(new Point(-60.000, -36.000),
                        new Point(-24.000, -36.000)))
                .build();

        assertEquals(0.0, robot1.startState().point.distanceTo(startPose.position), 1e-6);
    }

    @Test
    void exportedRobot2PathCompilesAndIsContinuous() {
        // --- pasted from the planner, unedited -----------------------------
        PathChain robot2 = new PathBuilder()
                .addPath(new BezierLine(new Point(60.000, 36.000),
                        new Point(20.000, 36.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(new Point(20.000, 36.000),
                        new Point(2.000, 36.000), new Point(-4.000, 56.000)))
                .setTangentHeadingInterpolation()
                .build();
        // -------------------------------------------------------------------

        assertEquals(2, robot2.size());

        // The planner keeps shared joints welded when you drag a handle; a chain
        // it exports should therefore never have a gap at a segment boundary.
        double gap = robot2.getSegment(0).getPath().getEndPoint()
                .distanceTo(robot2.getSegment(1).getPath().getStartPoint());
        assertEquals(0.0, gap, 1e-9);

        assertTrue(robot2.length() > 40.0);
    }

    @Test
    void anExportedPlanStaysInsideTheFieldWalls() {
        // Only checkable at all because the coordinates are absolute.
        PathChain robot1 = new PathBuilder()
                .addPath(new BezierLine(new Point(-60.000, -36.000),
                        new Point(-24.000, -36.000)))
                .addPath(new BezierCurve(new Point(-24.000, -36.000),
                        new Point(-6.000, -36.000), new Point(0.000, -12.000)))
                .build();

        for (double s = 0; s <= robot1.length(); s += 1.0) {
            assertTrue(FieldCoordinates.contains(robot1.stateAtArcLength(s).toPose(), 17, 17),
                    "a 17-inch robot should fit at s=" + s);
        }
    }

    @Test
    void anExportWithAPerSegmentSpeedCapCompilesAndApplies() {
        // --- pasted from the planner, unedited -----------------------------
        PathChain robot1 = new PathBuilder()
                .addPath(new BezierLine(new Point(-60.000, -36.000),
                        new Point(-24.000, -36.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(new Point(-24.000, -36.000),
                        new Point(-6.000, -36.000), new Point(0.000, -12.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0.00), Math.toRadians(90.00))
                .setMaxVelocity(8.000)
                .build();
        // -------------------------------------------------------------------

        assertTrue(robot1.hasVelocityOverrides());
        assertEquals(40.0, robot1.maxVelocityAtArcLength(10, 40), 1e-9,
                "the uncapped first segment keeps the global limit");
        assertEquals(8.0, robot1.maxVelocityAtArcLength(
                robot1.length() - 1, 40), 1e-9, "the capped segment is honoured");

        // And the cap has to actually cost time, or the planner's timeline is
        // predicting something the robot will not do.
        ConstrainedProfile capped = new ConstrainedProfile(robot1.length(),
                s -> robot1.maxVelocityAtArcLength(s, 40), 50, 60);
        ConstrainedProfile uncapped = new ConstrainedProfile(robot1.length(), 40, 50, 60);
        assertTrue(capped.duration() > uncapped.duration() * 1.5,
                "capping the second segment should dominate the run: "
                        + uncapped.duration() + "s -> " + capped.duration() + "s");
    }

    @Test
    void everyHeadingModeThePlannerCanEmitExists() {
        // One of each, so a renamed interpolator method is caught here rather
        // than by a team pasting the export in at a competition.
        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(new Point(-40, 0), new Point(-30, 0)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Point(-30, 0), new Point(-20, 0)))
                .setReverseTangentHeadingInterpolation()
                .addPath(new BezierLine(new Point(-20, 0), new Point(-10, 0)))
                .setConstantHeadingInterpolation(Math.toRadians(45.00))
                .addPath(new BezierLine(new Point(-10, 0), new Point(0, 0)))
                .setLinearHeadingInterpolation(Math.toRadians(0.00), Math.toRadians(90.00))
                .build();

        assertEquals(4, chain.size());
        assertEquals(0.0, chain.stateAtSegment(0, 0.5).heading, 1e-9);
        assertEquals(Math.PI, Math.abs(chain.stateAtSegment(1, 0.5).heading), 1e-9);
        assertEquals(Math.toRadians(45), chain.stateAtSegment(2, 0.5).heading, 1e-9);
        assertEquals(Math.toRadians(45), chain.stateAtSegment(3, 0.5).heading, 1e-9);
    }
}
