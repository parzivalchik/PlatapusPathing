package com.platapus.pathing;

import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.FieldCoordinates;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.kinematics.Kinematics;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.localization.FusedLocalizer;
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
 * The field-coordinate contract.
 *
 * <p>Paths are absolute: a point means the same spot whichever tile the robot
 * started on. The flip side is that the library has to be <i>told</i> where the
 * robot begins, and these tests pin down both halves of that.
 */
class FieldFrameTest {

    private static final double DT = 0.02;
    private static final double MAX_WHEEL_VELOCITY = 60.0;

    private MecanumKinematics kinematics() {
        return MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(MAX_WHEEL_VELOCITY).build();
    }

    private FollowerConstants constants() {
        return FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(30).maxAcceleration(40)
                .kV(1.0 / MAX_WHEEL_VELOCITY)
                .translationalPID(2.0, 0.0, 0.0)
                .headingPID(3.0, 0.0, 0.0)
                .positionTolerance(1.0)
                .headingTolerance(Math.toRadians(3))
                .build();
    }

    // --- FieldCoordinates ----------------------------------------------------

    @Test
    void theFieldIsTwelveFeetSquareCentredOnTheOrigin() {
        assertEquals(144.0, FieldCoordinates.FIELD_SIZE_INCHES, 1e-9);
        assertEquals(72.0, FieldCoordinates.HALF_FIELD_INCHES, 1e-9);
        assertEquals(24.0, FieldCoordinates.TILE_INCHES, 1e-9);
        assertTrue(FieldCoordinates.contains(new Vector2d(0, 0)));
        assertTrue(FieldCoordinates.contains(new Vector2d(72, -72)));
        assertFalse(FieldCoordinates.contains(new Vector2d(72.1, 0)));
    }

    @Test
    void fieldSizeConverts() {
        assertEquals(365.76, FieldCoordinates.fieldSize(DistanceUnit.CM), 1e-9);
    }

    @Test
    void aPointCanBeLegalWhileTheRobotStandingOnItIsNot() {
        // The reason there is a footprint-aware overload at all.
        Pose2d nearWall = new Pose2d(70, 0, 0);
        assertTrue(FieldCoordinates.contains(nearWall.position));
        assertFalse(FieldCoordinates.contains(nearWall, 18, 18),
                "an 18-inch robot centred 2 inches from the wall is through it");
        assertTrue(FieldCoordinates.contains(new Pose2d(60, 0, 0), 18, 18));
    }

    @Test
    void footprintContainmentAccountsForRotation() {
        // A long thin robot fits along one axis and not the other.
        Pose2d pose = new Pose2d(0, 66, 0);
        assertTrue(FieldCoordinates.contains(pose, 30, 8), "lengthwise along x, fits");
        assertFalse(FieldCoordinates.contains(new Pose2d(0, 66, Math.PI / 2), 30, 8),
                "turned to point at the wall, it does not");
    }

    @Test
    void rotating180MirrorsAPlanToTheOtherAlliance() {
        Pose2d red = new Pose2d(-60, -36, 0);
        Pose2d blue = FieldCoordinates.rotated180(red);

        assertEquals(60.0, blue.getX(), 1e-9);
        assertEquals(36.0, blue.getY(), 1e-9);
        assertEquals(Math.PI, Math.abs(blue.getHeading()), 1e-9);

        // And it is its own inverse, so mirroring twice is a no-op.
        assertTrue(FieldCoordinates.rotated180(blue).epsilonEquals(red, 1e-9, 1e-9));
    }

    @Test
    void clampingPullsAPointBackInsideTheWalls() {
        Vector2d clamped = FieldCoordinates.clampToField(new Vector2d(100, -90));
        assertEquals(72.0, clamped.x, 1e-9);
        assertEquals(-72.0, clamped.y, 1e-9);
    }

    // --- the localizer half --------------------------------------------------

    /** A drivetrain whose encoders the test drives directly. */
    private static final class Encoders implements Drivetrain {
        private final Kinematics kinematics;
        private final double[] positions;

        Encoders(Kinematics k) {
            this.kinematics = k;
            this.positions = new double[k.getWheelCount()];
        }

        void forward(double inches) {
            for (int i = 0; i < positions.length; i++) positions[i] += inches;
        }

        @Override public Kinematics getKinematics() { return kinematics; }
        @Override public void setWheelPowers(double[] powers) { }
        @Override public double[] getWheelPositions() { return positions.clone(); }
    }

    @Test
    void aFusedLocalizerStartsAtItsConfiguredFieldPose() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        Encoders encoders = new Encoders(kinematics());

        Pose2d start = new Pose2d(-60, -36, Math.PI / 2);
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .startPose(start)
                .build();

        assertTrue(localizer.getPose().epsilonEquals(start, 1e-9, 1e-9),
                "the pose must be the field start before any update runs");
    }

    @Test
    void fusedOdometryAccumulatesInTheFieldFrame() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        Encoders encoders = new Encoders(kinematics());

        // Facing field +Y, so driving forward increases y.
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .startPose(new Pose2d(-60, -36, Math.PI / 2))
                .build();
        localizer.update();

        for (int i = 0; i < 24; i++) {
            encoders.forward(1.0);
            clock.advance(DT);
            localizer.update();
        }

        assertEquals(-60.0, localizer.getPose().getX(), 1e-6);
        assertEquals(-12.0, localizer.getPose().getY(), 1e-6);
    }

    @Test
    void theGyroIsCalibratedAgainstTheStartHeadingNotItsOwnZero() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        Encoders encoders = new Encoders(kinematics());

        double[] gyro = {-2.7};   // whatever the IMU happens to read at init
        FusedLocalizer localizer = FusedLocalizer.builder(encoders, clock)
                .startPose(new Pose2d(0, 0, Math.toRadians(135)))
                .headingSource(() -> gyro[0])
                .headingVariance(1e-6)
                .build();
        localizer.update();

        for (int i = 0; i < 5; i++) { clock.advance(DT); localizer.update(); }

        assertEquals(Math.toRadians(135), localizer.getPose().getHeading(), 1e-3,
                "a stationary robot must hold its configured field heading");
    }

    @Test
    void startPoseMustNotBeNull() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        Encoders encoders = new Encoders(kinematics());
        assertThrows(IllegalArgumentException.class,
                () -> FusedLocalizer.builder(encoders, clock).startPose(null));
    }

    // --- the follower half ---------------------------------------------------

    @Test
    void theFollowerDrivesToAbsoluteFieldCoordinates() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        // The robot is placed away from the origin, and the path is absolute.
        Pose2d start = new Pose2d(-48, -24, 0);
        SimulatedRobot robot = new SimulatedRobot(kinematics(), clock, start);
        PathFollower follower = new PathFollower(robot, robot, constants(), clock);

        follower.followPath(PathChain.of(
                new BezierLine(new Point(-48, -24), new Point(-12, -24))));

        int loops = 0;
        while (follower.isBusy() && loops++ < 2000) {
            follower.update();
            robot.step(DT);
        }

        assertTrue(follower.isAtTarget(),
                "ended " + follower.getPositionError() + "\" away");
        assertEquals(-12.0, robot.getTruePose().getX(), 1.0);
        assertEquals(-24.0, robot.getTruePose().getY(), 1.0);
    }

    @Test
    void thePathDoesNotMoveWhenTheRobotStartsSomewhereElse() {
        // The point of absolute coordinates: the same chain targets the same
        // field spot regardless of where the robot is placed. Starting off-path
        // means the follower has to drive onto it -- which it should, rather
        // than silently translating the whole path.
        PathChain chain = PathChain.of(new BezierLine(new Point(0, 0), new Point(36, 0)));

        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        SimulatedRobot robot =
                new SimulatedRobot(kinematics(), clock, new Pose2d(0, -12, 0));
        PathFollower follower = new PathFollower(robot, robot, constants(), clock);

        follower.followPath(chain);
        int loops = 0;
        while (follower.isBusy() && loops++ < 2000) {
            follower.update();
            robot.step(DT);
        }

        // It converges on the path's absolute endpoint, not on (36, -12).
        assertEquals(36.0, robot.getTruePose().getX(), 1.5);
        assertEquals(0.0, robot.getTruePose().getY(), 1.5);
    }

    @Test
    void aMirroredPlanTargetsTheOppositeCornerOfTheField() {
        // Rotating a plan for the other alliance is a field-frame operation, and
        // it should need no rebuilding of the path.
        PathChain red = new PathBuilder()
                .addPath(new BezierLine(new Point(-60, -36), new Point(-24, -36)))
                .build();

        Vector2d redEnd = red.endState().point;
        Vector2d blueEnd = FieldCoordinates.rotated180(redEnd);

        assertEquals(24.0, blueEnd.x, 1e-3);
        assertEquals(36.0, blueEnd.y, 1e-3);
    }

    /** Whether an 18-inch robot fits everywhere along a chain. */
    private static boolean planFits(PathChain chain) {
        for (double s = 0; s <= chain.length(); s += 1.0) {
            if (!FieldCoordinates.contains(chain.stateAtArcLength(s).toPose(), 18, 18)) {
                return false;
            }
        }
        return true;
    }

    @Test
    void aWholePlanCanBeCheckedAgainstTheFieldWalls() {
        // Cheap sanity check a team can run on their own auto -- and something
        // only possible now that path points are absolute.

        // Centred 12" from the wall, an 18" robot reaches 69" against a 72"
        // wall: three inches of margin, so this is legal.
        assertTrue(planFits(new PathBuilder()
                .addPath(new BezierLine(new Point(-60, -60), new Point(60, -60)))
                .addPath(new BezierLine(new Point(60, -60), new Point(60, 60)))
                .build()));

        // Centred 4" from the wall it reaches 77", which is through it.
        assertFalse(planFits(PathChain.of(
                new BezierLine(new Point(-40, -68), new Point(40, -68)))),
                "a path 4 inches from the wall cannot fit an 18-inch robot");
    }

    @Test
    void containmentIsExactAtTheBoundary() {
        // Half-width 9 against a 72" wall: 63 is the last legal centre.
        assertTrue(FieldCoordinates.contains(new Pose2d(0, 63, 0), 18, 18));
        assertFalse(FieldCoordinates.contains(new Pose2d(0, 63.01, 0), 18, 18));
    }
}
