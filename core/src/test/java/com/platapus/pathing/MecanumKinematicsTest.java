package com.platapus.pathing;

import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Twist2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static com.platapus.pathing.kinematics.MecanumKinematics.BACK_LEFT;
import static com.platapus.pathing.kinematics.MecanumKinematics.BACK_RIGHT;
import static com.platapus.pathing.kinematics.MecanumKinematics.FRONT_LEFT;
import static com.platapus.pathing.kinematics.MecanumKinematics.FRONT_RIGHT;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MecanumKinematicsTest {

    private static final double EPS = 1e-9;

    /** trackWidth 15", wheelBase 13" -> turn radius (15+13)/2 = 14". */
    private final MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
            .trackWidth(15)
            .wheelBase(13)
            .maxWheelVelocity(60)
            .build();

    @Test
    void pureForwardDrivesAllFourWheelsTogether() {
        double[] w = kinematics.toWheelVelocities(new ChassisSpeeds(10, 0, 0));
        for (int i = 0; i < 4; i++) {
            assertEquals(10.0, w[i], EPS, "wheel " + i);
        }
    }

    @Test
    void pureStrafeFormsTheXPattern() {
        // Strafing left: the two wheels on one diagonal go forward, the other
        // two go backward.
        double[] w = kinematics.toWheelVelocities(new ChassisSpeeds(0, 10, 0));
        assertEquals(-10.0, w[FRONT_LEFT], EPS);
        assertEquals(10.0, w[FRONT_RIGHT], EPS);
        assertEquals(10.0, w[BACK_LEFT], EPS);
        assertEquals(-10.0, w[BACK_RIGHT], EPS);
    }

    @Test
    void pureRotationTurnsTheSidesAgainstEachOther() {
        // 1 rad/s CCW: left side backwards, right side forwards, at omega * 14".
        double[] w = kinematics.toWheelVelocities(new ChassisSpeeds(0, 0, 1));
        assertEquals(-14.0, w[FRONT_LEFT], EPS);
        assertEquals(14.0, w[FRONT_RIGHT], EPS);
        assertEquals(-14.0, w[BACK_LEFT], EPS);
        assertEquals(14.0, w[BACK_RIGHT], EPS);
    }

    @Test
    void forwardKinematicsInvertsInverseKinematics() {
        ChassisSpeeds[] cases = {
                new ChassisSpeeds(12, 0, 0),
                new ChassisSpeeds(0, -8, 0),
                new ChassisSpeeds(0, 0, 1.5),
                new ChassisSpeeds(20, 7, -0.9),
                new ChassisSpeeds(-5, 3, 0.25),
        };
        for (ChassisSpeeds in : cases) {
            ChassisSpeeds out = kinematics.toChassisSpeeds(kinematics.toWheelVelocities(in));
            assertEquals(in.vx, out.vx, 1e-9, "vx for " + in);
            assertEquals(in.vy, out.vy, 1e-9, "vy for " + in);
            assertEquals(in.omega, out.omega, 1e-9, "omega for " + in);
        }
    }

    @Test
    void roundTripHoldsWithImperfectStrafeEfficiency() {
        MecanumKinematics lossy = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60)
                .lateralEfficiency(0.8)
                .build();
        ChassisSpeeds in = new ChassisSpeeds(10, 6, 0.4);
        ChassisSpeeds out = lossy.toChassisSpeeds(lossy.toWheelVelocities(in));
        assertEquals(in.vx, out.vx, 1e-9);
        assertEquals(in.vy, out.vy, 1e-9);
        assertEquals(in.omega, out.omega, 1e-9);
    }

    @Test
    void lowerStrafeEfficiencyDemandsMoreWheelSpeedForTheSameStrafe() {
        MecanumKinematics lossy = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60)
                .lateralEfficiency(0.5)
                .build();
        double ideal = Math.abs(kinematics.toWheelVelocities(
                new ChassisSpeeds(0, 10, 0))[FRONT_RIGHT]);
        double compensated = Math.abs(lossy.toWheelVelocities(
                new ChassisSpeeds(0, 10, 0))[FRONT_RIGHT]);
        assertEquals(ideal * 2.0, compensated, EPS);
    }

    @Test
    void twistFromWheelDeltasMatchesTheVelocityCase() {
        double[] deltas = {1.0, 2.0, 1.5, 1.5};
        Twist2d twist = kinematics.toTwist(deltas);
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(deltas);
        assertEquals(speeds.vx, twist.dx, EPS);
        assertEquals(speeds.vy, twist.dy, EPS);
        assertEquals(speeds.omega, twist.dTheta, EPS);
    }

    @Test
    void normalizeLeavesInRangeCommandsUntouchedRelativeToTopSpeed() {
        double[] powers = kinematics.normalize(new double[]{60, 30, -30, 0});
        assertEquals(1.0, powers[0], EPS);
        assertEquals(0.5, powers[1], EPS);
        assertEquals(-0.5, powers[2], EPS);
        assertEquals(0.0, powers[3], EPS);
    }

    @Test
    void normalizeScalesTheWholeSetRatherThanClippingIndividualWheels() {
        // 120 exceeds the 60"/s ceiling; every wheel must shrink by the same
        // factor so the commanded direction survives.
        double[] powers = kinematics.normalize(new double[]{120, 60, -60, 0});
        assertEquals(1.0, powers[0], EPS);
        assertEquals(0.5, powers[1], EPS);
        assertEquals(-0.5, powers[2], EPS);
        assertEquals(0.0, powers[3], EPS);
    }

    @Test
    void normalizePreservesTheCommandedDirectionWhenSaturated() {
        ChassisSpeeds fast = new ChassisSpeeds(200, 90, 3);
        double[] raw = kinematics.toWheelVelocities(fast);
        double[] powers = kinematics.normalize(raw);

        ChassisSpeeds recovered = kinematics.toChassisSpeeds(powers);
        // Same direction in (vx, vy, omega) space, just smaller.
        double scale = recovered.vx / fast.vx;
        assertEquals(fast.vy * scale, recovered.vy, 1e-9);
        assertEquals(fast.omega * scale, recovered.omega, 1e-9);
        assertTrue(scale > 0 && scale < 1, "expected a shrink, got scale=" + scale);

        for (double p : powers) {
            assertTrue(Math.abs(p) <= 1.0 + EPS, "power out of range: " + p);
        }
    }

    @Test
    void maxAngularVelocityIsTopWheelSpeedOverTheTurnRadius() {
        assertEquals(60.0 / 14.0, kinematics.getMaxAngularVelocity(), EPS);
    }

    @Test
    void geometryGivenInCentimetresIsStoredInInches() {
        MecanumKinematics metric = MecanumKinematics.builder(DistanceUnit.CM)
                .trackWidth(38.1)   // 15 inches
                .wheelBase(33.02)   // 13 inches
                .maxWheelVelocity(152.4) // 60 in/s
                .build();
        assertEquals(15.0, metric.getTrackWidth(), 1e-9);
        assertEquals(13.0, metric.getWheelBase(), 1e-9);

        // And it therefore behaves identically to the inch-built one.
        double[] a = metric.toWheelVelocities(new ChassisSpeeds(0, 0, 1));
        double[] b = kinematics.toWheelVelocities(new ChassisSpeeds(0, 0, 1));
        for (int i = 0; i < 4; i++) {
            assertEquals(b[i], a[i], 1e-9, "wheel " + i);
        }
    }

    @Test
    void invalidGeometryIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> MecanumKinematics.builder(DistanceUnit.INCH)
                        .trackWidth(-1).wheelBase(13).maxWheelVelocity(60).build());
        assertThrows(IllegalArgumentException.class,
                () -> MecanumKinematics.builder(DistanceUnit.INCH)
                        .trackWidth(15).wheelBase(13).maxWheelVelocity(60)
                        .lateralEfficiency(1.5).build());
        assertThrows(IllegalStateException.class,
                () -> MecanumKinematics.builder(DistanceUnit.INCH).trackWidth(15).build());
    }

    @Test
    void wheelAccelerationsUseTheSameLinearMap() {
        // Named separately for readability, but it must stay the same map --
        // the kinematics are linear, so the derivative goes through unchanged.
        ChassisSpeeds acceleration = new ChassisSpeeds(30, -12, 1.4);
        double[] viaAccelerations = kinematics.toWheelAccelerations(acceleration);
        double[] viaVelocities = kinematics.toWheelVelocities(acceleration);

        assertArrayEquals(viaVelocities, viaAccelerations, 1e-12);
    }

    @Test
    void accelerationConversionIsLinearInTheChassisAcceleration() {
        // The property that makes reusing the velocity map legitimate at all.
        ChassisSpeeds a = new ChassisSpeeds(10, 4, 0.5);
        double[] once = kinematics.toWheelAccelerations(a);
        double[] thrice = kinematics.toWheelAccelerations(a.times(3));

        for (int i = 0; i < 4; i++) {
            assertEquals(once[i] * 3, thrice[i], 1e-12, "wheel " + i);
        }
    }

    @Test
    void wrongWheelCountIsRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> kinematics.toChassisSpeeds(new double[]{1, 2, 3}));
    }
}
