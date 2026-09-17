package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.ftc.MecanumDrivetrain;
import com.platapus.pathing.localization.OdometryComputer;
import com.platapus.pathing.localization.OdometryComputerLocalizer;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.units.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A complete worked example: build a path chain, follow it, report progress.
 *
 * <p>The numbers below describe a generic goBILDA-style mecanum robot. Replace
 * them with your own -- especially the motor names, which must match your robot
 * configuration exactly.
 *
 * <p>Remember the coordinate frame: coordinates are absolute field positions,
 * origin at the centre of the field. So {@code (-36, -36)} is a fixed spot, and
 * the library has to be told where the robot actually starts -- that is the
 * {@code startPose} below. See {@code FieldCoordinates}.
 */
@Autonomous(name = "PlatapusPathing Example Auto", group = "PlatapusPathing")
public class ExampleAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware -------------------------------------------------------

        MecanumDrivetrain drivetrain = MecanumDrivetrain.builder(DistanceUnit.INCH)
                .motors(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight")
                .reverse(true, false, true, false)
                .trackWidth(15.0)
                .wheelBase(13.0)
                .wheelRadius(1.89)          // 96 mm goBILDA mecanum
                .ticksPerRevolution(537.7)  // 312 RPM Yellow Jacket
                .maxMotorRpm(312)
                .build();
        drivetrain.resetEncoders();

        // Where the robot is actually placed, in field coordinates. Measure this
        // once against the tiles; get it wrong and the whole auto is offset by
        // the same amount.
        Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(0));

        // A goBILDA Pinpoint reading two odometry pods. It fuses them with its
        // own onboard gyro and hands back a finished pose, so nothing here
        // filters it further.
        //
        // PinpointOdometryComputer is not part of this library -- copy it from
        // examples/pinpoint/ into your TeamCode, alongside goBILDA's
        // GoBildaPinpointDriver.java, which is not published to Maven.
        OdometryComputer tracker = new PinpointOdometryComputer(
                hardwareMap, "pinpoint",
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                -84.0,    // X pod offset from the tracking centre, mm
                -168.0);  // Y pod offset from the tracking centre, mm

        OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
                .startPose(startPose)
                .build();

        // --- Tuning ---------------------------------------------------------

        FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(40)
                .maxAcceleration(50)
                .maxDeceleration(60)
                // Start from 1 / (top wheel speed in inches per second) and tune
                // from there. Phase 3's auto-tuner will fit these for you.
                .kV(0.017)
                .kS(0.05)
                .kA(0.0)
                .translationalPID(0.15, 0.0, 0.01)
                .headingPID(1.5, 0.0, 0.05)
                .positionTolerance(1.0)
                .headingTolerance(Math.toRadians(3))
                // Reactive blending: correct up to 2.5x as hard once the robot
                // is 4 inches or more off the path. Set reactiveAuthority(1.0)
                // to turn this off and get a plain profile follower back.
                .reactiveErrorScale(4.0)
                .reactiveAuthority(2.5)
                .build();

        PathFollower follower = new PathFollower(drivetrain, localizer, constants);

        // --- The path -------------------------------------------------------

        // Absolute field coordinates. The first point is where the robot starts,
        // so it does not have to drive onto the path before following it.
        PathChain chain = new PathBuilder()
                // Straight out two feet, turning to face field +Y as we go.
                .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                // Then curve away, holding that heading.
                .addPath(new BezierCurve(
                        new Point(-36, -36), new Point(-24, -24), new Point(-24, -6)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        telemetry.addData("Status", "Ready");
        telemetry.addData("Path", "%.1f inches over %d segments",
                chain.length(), chain.size());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // --- Follow ---------------------------------------------------------

        follower.followPath(chain);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Progress", "%.1f / %.1f in",
                    follower.getArcLengthTravelled(), chain.length());
            telemetry.addData("Error", "%.2f in, %.1f deg",
                    follower.getPositionError(),
                    AngleUnit.DEGREES.fromRadians(follower.getHeadingError()));
            // Worth watching: a pod that unplugs mid-auto still reports a
            // plausible-looking pose, so this is how you find out.
            telemetry.addData("Tracker", localizer.getHealthDetail());
            // Worth watching while tuning the blend: 1.0 means "tracking fine",
            // rising toward reactiveAuthority means "fighting to get back".
            telemetry.addData("Authority", "%.2f (confidence %.2f)",
                    follower.getCorrectionAuthority(), follower.getPoseConfidence());
            telemetry.update();
        }

        // The follower stops the motors itself, whether it arrived or timed out.
        telemetry.addData("Status", follower.isAtTarget() ? "Arrived" : "Timed out short");
        telemetry.addData("Final pose", follower.getPose());
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
