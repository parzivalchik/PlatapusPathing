package com.platapus.pathing;

import com.platapus.pathing.geometry.Angles;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.geometry.Vector2d;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class GeometryTest {

    private static final double EPS = 1e-9;

    @Test
    void unitConversionRoundTrips() {
        assertEquals(1.0, DistanceUnit.INCH.toInches(1.0), EPS);
        assertEquals(2.54, DistanceUnit.CM.fromInches(1.0), 1e-12);
        assertEquals(25.4, DistanceUnit.MM.fromInches(1.0), 1e-12);
        assertEquals(1.0, DistanceUnit.METER.fromInches(39.3700787401575), 1e-9);

        for (DistanceUnit unit : DistanceUnit.values()) {
            assertEquals(7.5, unit.fromInches(unit.toInches(7.5)), 1e-9, unit.name());
        }
        assertEquals(100.0, DistanceUnit.METER.convertTo(1.0, DistanceUnit.CM), 1e-9);
    }

    @Test
    void angleNormalizationWraps() {
        assertEquals(0.0, Angles.normalize(2 * Math.PI), 1e-12);
        assertEquals(Math.PI, Angles.normalize(Math.PI), 1e-12);
        assertEquals(Math.PI, Angles.normalize(-Math.PI), 1e-12);
        assertEquals(-Math.PI / 2, Angles.normalize(3 * Math.PI / 2), 1e-12);
        assertEquals(Math.PI / 2, Angles.normalizePositive(-3 * Math.PI / 2), 1e-12);
    }

    @Test
    void shortestInterpolationCrossesTheWrapPoint() {
        double result = Angles.lerpShortest(Math.toRadians(350), Math.toRadians(10), 0.5);
        assertEquals(0.0, Math.abs(result), 1e-12);
    }

    @Test
    void longestInterpolationAvoidsTheWrapPoint() {
        double result = Angles.lerpLongest(Math.toRadians(350), Math.toRadians(10), 0.5);
        assertEquals(Math.PI, Math.abs(result), 1e-12);
    }

    @Test
    void vectorRotationIsExact() {
        Vector2d rotated = new Vector2d(1, 0).rotated(Math.PI / 2);
        assertEquals(0.0, rotated.x, 1e-12);
        assertEquals(1.0, rotated.y, 1e-12);
    }

    @Test
    void degenerateVectorOperationsDoNotProduceNaN() {
        assertEquals(0.0, Vector2d.ZERO.normalized().norm(), EPS);
        assertEquals(0.0, new Vector2d(3, 4).projectOnto(Vector2d.ZERO).norm(), EPS);
    }

    @Test
    void transformByAndRelativeToAreInverses() {
        Pose2d outer = new Pose2d(10, -4, Math.toRadians(30));
        Pose2d inner = new Pose2d(3, 2, Math.toRadians(45));

        Pose2d composed = outer.transformBy(inner);
        Pose2d recovered = composed.relativeTo(outer);

        assertTrue(recovered.epsilonEquals(inner, 1e-9, 1e-9),
                "expected " + inner + " got " + recovered);
    }

    @Test
    void transformByLiftsAStartRelativePoseIntoAnOuterFrame() {
        // Start pose sits at (24, 24) facing +y in the outer frame. A robot
        // 10" "forward" of its start is therefore at (24, 34).
        Pose2d startInOuterFrame = new Pose2d(24, 24, Math.PI / 2);
        Pose2d lifted = startInOuterFrame.transformBy(new Pose2d(10, 0, 0));

        assertEquals(24.0, lifted.getX(), 1e-9);
        assertEquals(34.0, lifted.getY(), 1e-9);
        assertEquals(Math.PI / 2, lifted.getHeading(), 1e-9);
    }

    @Test
    void errorToIsExpressedInTheRobotFrame() {
        // Robot at the origin facing +y. A target at (0, 5) is 5" straight
        // ahead of it, so the robot-frame error is (5, 0).
        Pose2d pose = new Pose2d(0, 0, Math.PI / 2);
        Vector2d error = pose.errorTo(new Vector2d(0, 5));
        assertEquals(5.0, error.x, 1e-9);
        assertEquals(0.0, error.y, 1e-9);
    }

    @Test
    void headingErrorTakesTheShortWay() {
        Pose2d pose = new Pose2d(0, 0, Math.toRadians(350));
        assertEquals(Math.toRadians(20), pose.headingErrorTo(Math.toRadians(10)), 1e-9);
    }

    @Test
    void poseAccessorsConvertUnits() {
        Pose2d pose = Pose2d.of(100, 0, 0, DistanceUnit.CM);
        assertEquals(100.0 / 2.54, pose.getX(), 1e-9);
        assertEquals(100.0, pose.getX(DistanceUnit.CM), 1e-9);
    }

    @Test
    void straightTwistIsSimpleTranslation() {
        Pose2d result = new Twist2d(5, 0, 0).applyTo(Pose2d.ZERO);
        assertEquals(5.0, result.getX(), 1e-12);
        assertEquals(0.0, result.getY(), 1e-12);
    }

    @Test
    void curvedTwistIntegratesAsAnArcNotAChord() {
        // Driving forward while turning traces an arc. For a quarter turn of
        // radius r the robot ends at (r, r) relative to its start, not at the
        // (arcLength, 0) the naive integration would give.
        double radius = 10.0;
        double arcLength = radius * Math.PI / 2;
        Pose2d result = new Twist2d(arcLength, 0, Math.PI / 2).applyTo(Pose2d.ZERO);

        assertEquals(radius, result.getX(), 1e-9);
        assertEquals(radius, result.getY(), 1e-9);
        assertEquals(Math.PI / 2, result.getHeading(), 1e-12);
    }

    @Test
    void tinyRotationsUseTheTaylorExpansionWithoutBlowingUp() {
        Pose2d result = new Twist2d(1.0, 0, 1e-12).applyTo(Pose2d.ZERO);
        assertEquals(1.0, result.getX(), 1e-9);
        assertEquals(0.0, result.getY(), 1e-9);
        assertTrue(Double.isFinite(result.getY()));
    }

    @Test
    void manySmallTwistsAgreeWithOneLargeOne() {
        // The arc integration should be nearly exact, so splitting a movement
        // into pieces must not change where it ends up.
        Pose2d oneStep = new Twist2d(10, 0, 1.0).applyTo(Pose2d.ZERO);

        Pose2d manySteps = Pose2d.ZERO;
        int n = 1000;
        for (int i = 0; i < n; i++) {
            manySteps = new Twist2d(10.0 / n, 0, 1.0 / n).applyTo(manySteps);
        }

        assertTrue(oneStep.epsilonEquals(manySteps, 1e-6, 1e-9),
                "one step " + oneStep + " vs many " + manySteps);
    }

    @Test
    void fieldRelativeSpeedsRotateIntoTheRobotFrame() {
        // Robot facing +y, asked to move in world +y: that is straight forward.
        ChassisSpeeds speeds =
                ChassisSpeeds.fromFieldRelative(new Vector2d(0, 10), 0, Math.PI / 2);
        assertEquals(10.0, speeds.vx, 1e-9);
        assertEquals(0.0, speeds.vy, 1e-9);
    }
}
