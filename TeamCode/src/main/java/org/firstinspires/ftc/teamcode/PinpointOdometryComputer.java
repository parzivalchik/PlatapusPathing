/*
 * Adapts goBILDA's Pinpoint driver to this library's OdometryComputer.
 *
 * Both this file and GoBildaPinpointDriver.java live in TeamCode rather than in
 * :ftc, because the driver is not published to Maven -- goBILDA distribute it as
 * source. It is vendored here (MIT, Base 10 Assets LLC) so that this sample
 * actually compiles and runs; :core and :ftc have no dependency on it.
 *
 * Driver source: github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint
 *                branch goBILDA-Odometry-Driver
 * Re-copy that file to update; this adapter is the only thing that would break.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.localization.OdometryComputer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Adapts a goBILDA Pinpoint to {@link OdometryComputer}.
 *
 * <p>The Pinpoint reads two odometry pods, fuses them with its own onboard gyro,
 * and reports a finished pose. Note that its gyro is <b>not</b> the Control Hub's
 * -- it is a separate device on the Pinpoint board, and it is the reason this
 * library no longer ships a Control Hub IMU adapter at all.
 */
public final class PinpointOdometryComputer implements OdometryComputer {

    private final GoBildaPinpointDriver pinpoint;

    /**
     * @param hardwareMap  the OpMode's hardware map
     * @param name         the Pinpoint's name in your robot configuration
     * @param pods         which goBILDA pods you have, so the driver knows the
     *                     encoder resolution
     * @param xOffsetMm    how far the X (forward) pod sits from the tracking
     *                     centre, millimetres, left of centre positive
     * @param yOffsetMm    how far the Y (strafe) pod sits from the tracking
     *                     centre, millimetres, forward of centre positive
     */
    public PinpointOdometryComputer(HardwareMap hardwareMap, String name,
                                    GoBildaPinpointDriver.GoBildaOdometryPods pods,
                                    double xOffsetMm, double yOffsetMm) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, name);
        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderResolution(pods);

        // Both FORWARD is only a starting point. Verify it during bring-up:
        // push the robot forward by hand and confirm x rises, push it left and
        // confirm y rises. A reversed pod looks exactly like a robot that
        // drives backwards for no reason.
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Zeroes the pose and recalibrates the gyro. Takes a moment, and the
        // robot must be still for it, so do this in init and not after start.
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void refresh() {
        pinpoint.update();
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public ChassisSpeeds getVelocity() {
        // The driver reports velocity in the field frame, alongside its field
        // position, so it is rotated into the robot frame here -- which is what
        // OdometryComputer asks for. Confirm during bring-up: driving straight
        // forward should give a positive vx and a vy near zero whichever way
        // the robot is pointing. If vy swings about instead, drop the rotation.
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        double fieldVx = pinpoint.getVelX(DistanceUnit.INCH);
        double fieldVy = pinpoint.getVelY(DistanceUnit.INCH);

        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        return new ChassisSpeeds(
                fieldVx * cos - fieldVy * sin,
                fieldVx * sin + fieldVy * cos,
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }

    @Override
    public void setPose(Pose2d pose) {
        pinpoint.setPosX(pose.getX(), DistanceUnit.INCH);
        pinpoint.setPosY(pose.getY(), DistanceUnit.INCH);
        pinpoint.setHeading(pose.getHeading(), AngleUnit.RADIANS);
    }

    @Override
    public Health getHealth() {
        switch (pinpoint.getDeviceStatus()) {
            case READY:
                return Health.READY;
            case NOT_READY:
            case CALIBRATING:
                return Health.CALIBRATING;
            case FAULT_X_POD_NOT_DETECTED:
            case FAULT_Y_POD_NOT_DETECTED:
            case FAULT_NO_PODS_DETECTED:
            case FAULT_IMU_RUNAWAY:
            case FAULT_BAD_READ:
                return Health.FAULT;
            default:
                return Health.UNKNOWN;
        }
    }

    @Override
    public String getHealthDetail() {
        // The vendor's own status name is far more useful on telemetry than our
        // coarse one: "FAULT_X_POD_NOT_DETECTED" tells you which cable to check.
        return pinpoint.getDeviceStatus().name();
    }

    /** The underlying driver, for anything this adapter does not expose. */
    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }
}
