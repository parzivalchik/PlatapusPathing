package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.platapus.pathing.command.CommandRunner;
import com.platapus.pathing.command.Commands;
import com.platapus.pathing.command.FollowPathCommand;
import com.platapus.pathing.control.Clock;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.ftc.FtcTuningStore;
import com.platapus.pathing.ftc.HubVoltageSource;
import com.platapus.pathing.ftc.MecanumDrivetrain;
import com.platapus.pathing.localization.OdometryComputer;
import com.platapus.pathing.localization.OdometryComputerLocalizer;
import com.platapus.pathing.paths.BezierCurve;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathBuilder;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.StallEvent;
import com.platapus.pathing.tuning.TunableFeedforward;
import com.platapus.pathing.tuning.StallDetector;
import com.platapus.pathing.tuning.TuningSession;
import com.platapus.pathing.units.DistanceUnit;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Everything from Phase 3 together: the command layer sequencing an auto, the
 * feedforward fitting itself online and persisting between runs, and per-motor
 * stall detection reporting to the OpMode.
 *
 * <p>{@link ExampleAutoOpMode} is the simpler starting point -- read that first.
 */
@Autonomous(name = "PlatapusPathing Tuned Auto", group = "PlatapusPathing")
public class ExampleTunedAutoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Clock clock = Clock.system();

        // --- Hardware -------------------------------------------------------

        MecanumDrivetrain drivetrain = MecanumDrivetrain.builder(DistanceUnit.INCH)
                .motors(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight")
                .reverse(true, false, true, false)
                .trackWidth(15.0).wheelBase(13.0).wheelRadius(1.89)
                .ticksPerRevolution(537.7).maxMotorRpm(312)
                .build();
        drivetrain.resetEncoders();

        // Field coordinates, so the robot's real starting spot has to be given.
        Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(0));

        // See ExampleAutoOpMode for where PinpointOdometryComputer comes from.
        OdometryComputer tracker = new PinpointOdometryComputer(
                hardwareMap, "pinpoint",
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD,
                -84.0, -168.0);

        OdometryComputerLocalizer localizer = OdometryComputerLocalizer.builder(tracker)
                .startPose(startPose)
                .build();

        HubVoltageSource voltage = new HubVoltageSource(hardwareMap);

        // --- Feedforward, fitted online and remembered between runs ----------

        // The compiled-in guess. Only used the very first time this robot ever
        // runs; after that the saved fit takes over.
        TunableFeedforward feedforward =
                new TunableFeedforward(new FeedforwardGains(0.05, 0.017, 0.0));

        FtcTuningStore store = new FtcTuningStore();
        TuningSession tuning = TuningSession.builder(drivetrain, voltage, clock, feedforward)
                .forgettingFactor(0.99)
                .store(store)
                .build();

        boolean restored = tuning.restore();

        FollowerConstants constants = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(40).maxAcceleration(50).maxDeceleration(60)
                // kS/kV/kA here are the fallback; the TunableFeedforward below
                // is what the follower actually reads each loop.
                .kV(0.017).kS(0.05)
                .translationalPID(0.15, 0.0, 0.01)
                .headingPID(1.5, 0.0, 0.05)
                .reactiveErrorScale(4.0).reactiveAuthority(2.5)
                .build();

        PathFollower follower =
                new PathFollower(drivetrain, localizer, constants, clock, feedforward);

        // --- Stall detection --------------------------------------------------

        AtomicReference<String> lastStall = new AtomicReference<>("none");
        StallDetector stallDetector = StallDetector.builder(feedforward, voltage)
                .stallCurrentAmps(9.2)      // goBILDA Yellow Jacket, from the datasheet
                .currentRatioThreshold(2.0)
                .debounceSeconds(0.25)
                .build();

        // The library reports; you decide what it means. Aborting the path,
        // backing off and retrying, or simply logging it are all reasonable and
        // all game-specific, so none of them are built in.
        stallDetector.addListener(new com.platapus.pathing.tuning.StallListener() {
            @Override
            public void onStallDetected(StallEvent event) {
                lastStall.set(String.format("motor %d @ %.1fs (%.1fA vs %.1fA)",
                        event.motorIndex, event.timestamp,
                        event.measuredCurrent, event.expectedCurrent));
            }

            @Override
            public void onStallCleared(int motorIndex, double timestamp) {
                lastStall.set("cleared motor " + motorIndex);
            }
        });

        // --- The auto ---------------------------------------------------------

        PathChain toScore = new PathBuilder()
                .addPath(new BezierLine(new Point(-60, -36), new Point(-36, -36)))
                .setLinearHeadingInterpolation(0, Math.toRadians(90))
                .build();

        PathChain toPark = new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(-36, -36), new Point(-24, -24), new Point(-24, -6)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        CommandRunner runner = new CommandRunner(Commands.sequence(
                new FollowPathCommand(follower, toScore),
                Commands.run(() -> telemetry.log().add("scored")),
                Commands.waitSeconds(0.3, clock),
                new FollowPathCommand(follower, toPark)));

        telemetry.addData("Feedforward", restored
                ? "restored from " + store.getFile().getName()
                : "using compiled-in defaults (nothing saved yet)");
        telemetry.addData("Gains", feedforward.get());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive() && !runner.isFinished()) {
            runner.run();

            double[] powers = follower.getLastPowers();
            // The tuner learns from whatever the robot happens to be doing --
            // there is no separate tuning routine to run.
            tuning.update(powers);
            // Stall detection compares measured current against what the
            // *commanded* wheel speeds should have cost.
            stallDetector.update(clock.seconds(), powers,
                    follower.getLastWheelVelocities(),
                    drivetrain.getMotorCurrents());

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Authority", "%.2f", follower.getCorrectionAuthority());
            telemetry.addData("Fit", "%s (conf %.2f, n=%d)",
                    tuning.getFittedGains(), tuning.getTuner().getConfidence(),
                    tuning.getTuner().getSampleCount());
            telemetry.addData("Pending", feedforward.hasPending()
                    ? "yes -- applies at the next path" : "no");
            telemetry.addData("Stall", lastStall.get());
            telemetry.addData("Tracker", localizer.getHealthDetail());
            telemetry.update();
        }

        runner.cancel();

        // Save what the follower actually used, so the next run starts here.
        boolean saved = tuning.persist();
        telemetry.addData("Saved", saved ? feedforward.get().toString() : "failed");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
