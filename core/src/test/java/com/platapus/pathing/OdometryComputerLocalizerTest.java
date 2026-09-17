package com.platapus.pathing;

import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.localization.OdometryComputer;
import com.platapus.pathing.localization.OdometryComputerLocalizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The external-pose-tracker path: a goBILDA Pinpoint or similar, faked so the
 * behaviour that matters -- what happens when a pod falls out -- is testable
 * without the hardware.
 */
class OdometryComputerLocalizerTest {

    /** A scriptable stand-in for a Pinpoint. */
    private static final class FakeComputer implements OdometryComputer {
        Pose2d pose = Pose2d.ZERO;
        ChassisSpeeds velocity = ChassisSpeeds.ZERO;
        Health health = Health.READY;
        String detail = "READY";
        int refreshes;
        Pose2d lastWritten;

        @Override public void refresh() { refreshes++; }
        @Override public Pose2d getPose() { return pose; }
        @Override public ChassisSpeeds getVelocity() { return velocity; }
        @Override public void setPose(Pose2d p) { pose = p; lastWritten = p; }
        @Override public Health getHealth() { return health; }
        @Override public String getHealthDetail() { return detail; }
    }

    private OdometryComputerLocalizer localizerAt(FakeComputer device, Pose2d start) {
        return OdometryComputerLocalizer.builder(device).startPose(start).build();
    }

    @Test
    void theStartPoseIsWrittenToTheDeviceNotJustRemembered() {
        // The device keeps its own pose across OpModes, so leaving it alone
        // would silently inherit wherever the last run finished.
        FakeComputer device = new FakeComputer();
        device.pose = new Pose2d(999, 999, 3);

        Pose2d start = new Pose2d(-60, -36, Math.PI / 2);
        OdometryComputerLocalizer localizer = localizerAt(device, start);

        assertTrue(device.lastWritten.epsilonEquals(start, 1e-9, 1e-9),
                "the device should have been told where the robot is");
        assertTrue(localizer.getPose().epsilonEquals(start, 1e-9, 1e-9));
    }

    @Test
    void thePoseIsPassedStraightThroughWithNoFiltering() {
        // The device has already fused pods and gyro; smoothing it again would
        // add lag to an estimate better than anything we could add.
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);

        device.pose = new Pose2d(12.345, -6.789, 1.234);
        localizer.update();

        assertEquals(12.345, localizer.getPose().getX(), 1e-12);
        assertEquals(-6.789, localizer.getPose().getY(), 1e-12);
        assertEquals(1.234, localizer.getPose().getHeading(), 1e-12);
    }

    @Test
    void refreshHappensExactlyOncePerUpdate() {
        // These are I2C reads; doing two per loop halves the loop rate.
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);

        for (int i = 0; i < 5; i++) {
            localizer.update();
            localizer.getPose();
            localizer.getVelocity();
        }
        assertEquals(5, device.refreshes);
    }

    @Test
    void theTwistIsTheRobotFrameMovementBetweenReadings() {
        FakeComputer device = new FakeComputer();
        // Facing field +Y. Moving +10 in field Y is 10 inches straight forward.
        OdometryComputerLocalizer localizer =
                localizerAt(device, new Pose2d(0, 0, Math.PI / 2));

        device.pose = new Pose2d(0, 10, Math.PI / 2);
        localizer.update();

        assertEquals(10.0, localizer.getLastTwist().dx, 1e-9, "forward");
        assertEquals(0.0, localizer.getLastTwist().dy, 1e-9, "no sideways");
        assertEquals(0.0, localizer.getLastTwist().dTheta, 1e-9);
    }

    @Test
    void aHealthyDeviceIsFullyTrusted() {
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);
        localizer.update();

        assertEquals(1.0, localizer.getConfidence(), 1e-12);
        assertEquals(OdometryComputer.Health.READY, localizer.getHealth());
        assertFalse(localizer.hasFaulted());
    }

    // --- the failure modes drive encoders do not have ------------------------

    @Test
    void aFaultDropsConfidenceToZeroSoTheFollowerBacksOff() {
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);
        localizer.update();

        device.health = OdometryComputer.Health.FAULT;
        device.detail = "FAULT_X_POD_NOT_DETECTED";
        localizer.update();

        assertEquals(0.0, localizer.getConfidence(), 1e-12,
                "a faulted tracker must not be driven toward at full authority");
        assertEquals("FAULT_X_POD_NOT_DETECTED", localizer.getHealthDetail());
    }

    @Test
    void aFaultHoldsTheLastGoodPoseRatherThanAcceptingTheReading() {
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);

        device.pose = new Pose2d(24, 12, 0.5);
        localizer.update();

        // The pod falls out and the device starts reporting nonsense.
        device.health = OdometryComputer.Health.FAULT;
        device.pose = new Pose2d(-5000, 91000, 42);
        localizer.update();

        assertEquals(24.0, localizer.getPose().getX(), 1e-9);
        assertEquals(12.0, localizer.getPose().getY(), 1e-9);
        assertEquals(0.0, localizer.getVelocity().vx, 1e-12,
                "velocity must not be believed either");
    }

    @Test
    void aNonFiniteReadingIsTreatedAsAFaultEvenIfTheDeviceSaysReady() {
        // The dangerous case: the device claims health while handing back NaN.
        // A NaN reaching the follower turns every motor command into NaN.
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);

        device.pose = new Pose2d(10, 0, 0);
        localizer.update();

        device.pose = new Pose2d(Double.NaN, 0, 0);
        localizer.update();

        assertEquals(10.0, localizer.getPose().getX(), 1e-9, "the NaN must not propagate");
        assertEquals(OdometryComputer.Health.FAULT, localizer.getHealth());
        assertEquals(0.0, localizer.getConfidence(), 1e-12);
    }

    @Test
    void calibratingIsNotTrustedButIsNotAFaultEither() {
        FakeComputer device = new FakeComputer();
        device.health = OdometryComputer.Health.CALIBRATING;
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);
        localizer.update();

        assertEquals(0.0, localizer.getConfidence(), 1e-12);
        assertFalse(localizer.hasFaulted(),
                "powering up is not a failure to latch");
    }

    @Test
    void aFaultLatchesBecauseRecoveringDoesNotUndoTheDriftItCaused() {
        // A pod that reconnects leaves the pose offset by however far the robot
        // moved while it was out, and the device reports READY again regardless.
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);
        localizer.update();

        device.health = OdometryComputer.Health.FAULT;
        localizer.update();
        device.health = OdometryComputer.Health.READY;
        localizer.update();

        assertEquals(1.0, localizer.getConfidence(), 1e-12, "trusted again");
        assertTrue(localizer.hasFaulted(), "but the OpMode should still be able to know");

        localizer.clearFaultLatch();
        assertFalse(localizer.hasFaulted());
    }

    @Test
    void resumingAfterAFaultDoesNotProduceAHugePhantomTwist() {
        // The twist is measured against the last pose we believed, so a gap in
        // readings must not look like the robot teleported.
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);

        device.pose = new Pose2d(10, 0, 0);
        localizer.update();

        device.health = OdometryComputer.Health.FAULT;
        localizer.update();
        assertEquals(0.0, localizer.getLastTwist().dx, 1e-12,
                "no movement should be reported while faulted");

        device.health = OdometryComputer.Health.READY;
        device.pose = new Pose2d(11, 0, 0);
        localizer.update();
        assertEquals(1.0, localizer.getLastTwist().dx, 1e-9,
                "and the first good reading is measured from the last good one");
    }

    @Test
    void setPoseAlsoRebasesTheDevice() {
        FakeComputer device = new FakeComputer();
        OdometryComputerLocalizer localizer = localizerAt(device, Pose2d.ZERO);

        Pose2d rebased = new Pose2d(30, -18, Math.PI);
        localizer.setPose(rebased);

        assertTrue(device.lastWritten.epsilonEquals(rebased, 1e-9, 1e-9),
                "rebasing only our copy would leave the device disagreeing");
        assertTrue(localizer.getPose().epsilonEquals(rebased, 1e-9, 1e-9));
    }

    @Test
    void nullArgumentsAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> OdometryComputerLocalizer.builder(null));
        assertThrows(IllegalArgumentException.class,
                () -> OdometryComputerLocalizer.builder(new FakeComputer()).startPose(null));
        assertThrows(IllegalArgumentException.class,
                () -> localizerAt(new FakeComputer(), Pose2d.ZERO).setPose(null));
    }
}
