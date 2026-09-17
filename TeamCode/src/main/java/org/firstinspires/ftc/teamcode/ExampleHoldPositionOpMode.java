package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.platapus.pathing.command.CommandRunner;
import com.platapus.pathing.command.Commands;
import com.platapus.pathing.command.FollowPathCommand;
import com.platapus.pathing.command.HoldPositionCommand;
import com.platapus.pathing.control.Clock;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.follower.PoseHolder;
import com.platapus.pathing.ftc.MecanumDrivetrain;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.localization.OdometryComputer;
import com.platapus.pathing.localization.OdometryComputerLocalizer;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.units.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Holding a pose: drive somewhere, stay put while a mechanism runs, drive on.
 *
 * <p>{@link PathFollower} stops when its profile ends -- an auto that fought to
 * stay on its last waypoint for the rest of the match would be a surprise, not a
 * feature. {@link PoseHolder} is the opposite tool, and this shows both ways of
 * reaching for it.
 *
 * <p>{@link ExampleAutoOpMode} is the simpler starting point; read that first.
 */
@Autonomous(name = "PlatapusPathing Hold Position", group = "PlatapusPathing")
public class ExampleHoldPositionOpMode extends LinearOpMode {

    /** Stands in for whatever mechanism you are actually waiting on. */
    private double mechanismFinishesAt = Double.MAX_VALUE;

    @Override
    public void runOpMode() throws InterruptedException {
        Clock clock = Clock.system();

        MecanumDrivetrain drivetrain = MecanumDrivetrain.builder(DistanceUnit.INCH)
                .motors(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight")
                .reverse(true, false, true, false)
                .trackWidth(15.0).wheelBase(13.0).wheelRadius(1.89)
                .ticksPerRevolution(537.7).maxMotorRpm(312)
                .build();
        drivetrain.resetEncoders();

        Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(0));

        OdometryComputer tracker = new PinpointOdometryComputer(
                hardwareMap, "pinpoint",
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                -84.0, -168.0);

        OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
                .startPose(startPose)
                .build();

        FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(40).maxAcceleration(50).maxDeceleration(60)
                .kV(0.017).kS(0.05)
                .translationalPID(0.15, 0.0, 0.01)
                .headingPID(1.5, 0.0, 0.05)
                // Tighter than the follower's default: a hold is judged on where
                // it parks, and the tolerance is also the deadband inside which
                // PoseHolder cuts power rather than hunting.
                .positionTolerance(0.75)
                .headingTolerance(Math.toRadians(2))
                .build();

        PathFollower follower = new PathFollower(drivetrain, localizer, constants, clock);

        // The holder shares the drivetrain, localizer and gains with the
        // follower. Only ever run one of them at a time -- they both write motor
        // powers, and two controllers fighting over the same actuator is the
        // classic way to make a robot shudder.
        PoseHolder holder = new PoseHolder(drivetrain, localizer, constants, clock);

        Pose2d scoringPose = new Pose2d(-36, -36, Math.toRadians(90));

        PathChain toScoring = new PathBuilder()
                .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                .build();

        PathChain toPark = PathChain.of(
                new BezierLine(new Point(-36, -36), new Point(-12, -36)));

        CommandRunner runner = new CommandRunner(Commands.sequence(
                new FollowPathCommand(follower, toScoring),

                // Hold the scoring pose for exactly as long as the mechanism
                // takes. deadline() ends when its FIRST command ends, so the
                // hold lasts precisely as long as the work does -- a hold has no
                // natural end of its own, and inside a plain sequence it would
                // never let the auto advance.
                Commands.deadline(
                        Commands.sequence(
                                Commands.run(this::startMechanism),
                                Commands.waitUntil(this::mechanismDone)),
                        new HoldPositionCommand(holder, scoringPose)),

                new FollowPathCommand(follower, toPark),

                // Park and stay parked for the rest of the auto, resisting
                // anything that leans on us. holdCurrentPose captures wherever
                // the follower actually stopped rather than where it aimed.
                HoldPositionCommand.holdCurrentPose(holder, () -> !opModeIsActive())));

        telemetry.addData("Status", "Ready");
        telemetry.addData("Tracker", localizer.getHealthDetail());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !runner.isFinished()) {
            runner.run();

            telemetry.addData("Pose", follower.getPose());
            if (holder.isHolding()) {
                telemetry.addData("Holding", holder.getTarget());
                telemetry.addData("Hold error", "%.2f in, %.1f deg",
                        holder.getPositionError(),
                        AngleUnit.DEGREES.fromRadians(holder.getHeadingError()));
                // False here while parked means something is pushing us and the
                // holder is working; it is not an error.
                telemetry.addData("On target", holder.isAtTarget());
            }
            telemetry.addData("Tracker", localizer.getHealthDetail());
            telemetry.update();
        }

        // Both write motor powers, so both get told to stop.
        runner.cancel();
        follower.breakFollowing();
        holder.stop();
    }

    private void startMechanism() {
        // Replace with your arm, lift or claw. Two seconds of pretending.
        mechanismFinishesAt = System.nanoTime() * 1e-9 + 2.0;
    }

    private boolean mechanismDone() {
        return System.nanoTime() * 1e-9 >= mechanismFinishesAt;
    }
}
