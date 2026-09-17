package com.platapus.pathing;

import com.platapus.pathing.command.Command;
import com.platapus.pathing.command.CommandRunner;
import com.platapus.pathing.command.Commands;
import com.platapus.pathing.command.HoldPositionCommand;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PoseHolder;
import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PoseHolderTest {

    private static final double DT = 0.02;
    private static final double MAX_WHEEL_VELOCITY = 60.0;

    private SimulatedRobot.ManualClock clock;
    private SimulatedRobot robot;
    private PoseHolder holder;

    @BeforeEach
    void setUp() {
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(MAX_WHEEL_VELOCITY).build();
        clock = new SimulatedRobot.ManualClock();
        robot = new SimulatedRobot(kinematics, clock, new Pose2d(-36, -36, 0));
        holder = new PoseHolder(robot, robot, constants(), clock);
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

    private void run(int loops) {
        for (int i = 0; i < loops; i++) {
            holder.update();
            robot.step(DT);
        }
    }

    @Test
    void itDoesNothingUntilToldToHold() {
        run(10);
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-12);
        }
        assertFalse(holder.isHolding());
    }

    @Test
    void holdingWhereItAlreadyIsCommandsNothing() {
        // The most common case -- parked on target -- must be silent, not a
        // motor buzzing against its own tolerance.
        holder.hold(new Pose2d(-36, -36, 0));
        run(25);

        assertTrue(holder.isAtTarget());
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-12, "should not hunt while on target");
        }
    }

    @Test
    void itDrivesBackAfterBeingShoved() {
        holder.hold(new Pose2d(-36, -36, 0));
        run(5);

        robot.displace(6, -4, 0);
        run(400);

        assertEquals(-36.0, robot.getTruePose().getX(), 1.0);
        assertEquals(-36.0, robot.getTruePose().getY(), 1.0);
        assertTrue(holder.isAtTarget(),
                "ended " + holder.getPositionError() + "\" away");
    }

    @Test
    void itTurnsBackAfterBeingRotated() {
        holder.hold(new Pose2d(-36, -36, 0));
        run(5);

        robot.displace(0, 0, Math.toRadians(40));
        run(400);

        assertEquals(0.0, robot.getTruePose().getHeading(), Math.toRadians(4));
    }

    @Test
    void itKeepsResistingASteadyPush() {
        // A partner robot leaning on you: a follower would have finished and
        // gone limp, which is the whole reason this class exists.
        holder.hold(new Pose2d(-36, -36, 0));
        robot.setDisturbance(new ChassisSpeeds(0, 3.0, 0));

        double worst = 0;
        for (int i = 0; i < 500; i++) {
            holder.update();
            robot.step(DT);
            worst = Math.max(worst,
                    robot.getTruePose().position.distanceTo(new Pose2d(-36, -36, 0).position));
        }

        assertTrue(worst < 5.0, "drifted " + worst + "\" under a constant push");
        assertTrue(holder.isHolding(), "a hold has no natural end");
    }

    @Test
    void itHoldsAPoseTheRobotIsNotAtYet() {
        holder.hold(new Pose2d(-24, -36, Math.toRadians(45)));
        run(600);

        assertEquals(-24.0, robot.getTruePose().getX(), 1.5);
        assertEquals(Math.toRadians(45), robot.getTruePose().getHeading(),
                Math.toRadians(5));
    }

    @Test
    void holdCurrentPoseCapturesWhereItIsNow() {
        robot.setPose(new Pose2d(10, -20, Math.toRadians(30)));
        holder.holdCurrentPose();

        assertEquals(10.0, holder.getTarget().getX(), 1e-9);
        assertEquals(-20.0, holder.getTarget().getY(), 1e-9);
        assertEquals(Math.toRadians(30), holder.getTarget().getHeading(), 1e-9);
    }

    @Test
    void stoppingCutsThePowerAndEndsTheHold() {
        holder.hold(new Pose2d(-24, -36, 0));
        run(10);
        assertTrue(holder.isHolding());

        holder.stop();

        assertFalse(holder.isHolding());
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-12);
        }
    }

    @Test
    void aNewHoldDoesNotInheritTheOldOnesIntegral() {
        FollowerConstants withIntegral = FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(30).maxAcceleration(40)
                .kV(1.0 / MAX_WHEEL_VELOCITY)
                .translationalPID(1.0, 0.5, 0.0)
                .headingPID(3.0, 0.0, 0.0)
                .positionTolerance(1.0).headingTolerance(Math.toRadians(3))
                .build();
        PoseHolder integrating = new PoseHolder(robot, robot, withIntegral, clock);

        // Wind the integral up against a target it cannot reach.
        robot.setPowerScale(0.02);
        integrating.hold(new Pose2d(40, 40, 0));
        for (int i = 0; i < 200; i++) { integrating.update(); robot.step(DT); }
        robot.setPowerScale(1.0);

        // A fresh hold on the spot must not lurch from that accumulated term.
        integrating.hold(robot.getTruePose());
        integrating.update();
        for (double p : robot.getLastPowers()) {
            assertTrue(Math.abs(p) < 0.1, "lurched with power " + p);
        }
    }

    @Test
    void powersStayInRangeAndNeverGoNaN() {
        holder.hold(new Pose2d(60, 60, Math.PI));
        for (int i = 0; i < 300; i++) {
            holder.update();
            for (double p : holder.getLastPowers()) {
                assertTrue(Math.abs(p) <= 1.0 + 1e-9, "out of range: " + p);
                assertFalse(Double.isNaN(p), "NaN power");
            }
            robot.step(DT);
        }
    }

    @Test
    void aNullTargetIsRejected() {
        assertThrows(IllegalArgumentException.class, () -> holder.hold(null));
    }

    // --- as a command --------------------------------------------------------

    @Test
    void theCommandHoldsForTheLifetimeOfADeadline() {
        boolean[] workDone = {false};

        Command work = Commands.sequence(
                Commands.waitSeconds(0.4, clock),
                Commands.run(() -> workDone[0] = true));

        CommandRunner runner = new CommandRunner(Commands.deadline(
                work, new HoldPositionCommand(holder, new Pose2d(-36, -36, 0))));

        int loops = 0;
        while (!runner.isFinished() && loops++ < 500) {
            runner.run();
            robot.step(DT);
            clock.advance(0);   // the sim advances the clock itself
        }

        assertTrue(workDone[0], "the deadline work should have finished");
        assertFalse(holder.isHolding(), "and the hold should have been ended");
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-12, "motors must be released afterwards");
        }
    }

    @Test
    void theCommandEndsWhenItsConditionIsMet() {
        boolean[] release = {false};
        CommandRunner runner = new CommandRunner(
                new HoldPositionCommand(holder, new Pose2d(-36, -36, 0), () -> release[0]));

        for (int i = 0; i < 20; i++) { runner.run(); robot.step(DT); }
        assertFalse(runner.isFinished());
        assertTrue(holder.isHolding());

        release[0] = true;
        runner.run();

        assertTrue(runner.isFinished());
        assertFalse(holder.isHolding());
    }

    @Test
    void anInterruptedHoldReleasesTheMotors() {
        CommandRunner runner = new CommandRunner(
                new HoldPositionCommand(holder, new Pose2d(-24, -36, 0)));

        for (int i = 0; i < 10; i++) { runner.run(); robot.step(DT); }
        runner.cancel();

        assertFalse(holder.isHolding());
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-12);
        }
    }
}
