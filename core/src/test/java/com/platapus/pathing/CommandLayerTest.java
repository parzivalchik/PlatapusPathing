package com.platapus.pathing;

import com.platapus.pathing.command.Command;
import com.platapus.pathing.command.CommandRunner;
import com.platapus.pathing.command.Commands;
import com.platapus.pathing.command.FollowPathCommand;
import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CommandLayerTest {

    /** Records its lifecycle so tests can assert on ordering. */
    private static final class Recorder implements Command {
        final String name;
        final List<String> log;
        int executions;
        int finishAfter;

        Recorder(String name, List<String> log, int finishAfter) {
            this.name = name;
            this.log = log;
            this.finishAfter = finishAfter;
        }

        @Override public void initialize() { log.add(name + ":init"); }
        @Override public void execute() { executions++; log.add(name + ":exec"); }
        @Override public boolean isFinished() { return executions >= finishAfter; }
        @Override public void end(boolean interrupted) {
            log.add(name + (interrupted ? ":interrupted" : ":end"));
        }
    }

    @Test
    void runnerDrivesTheFullLifecycleInOrder() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(new Recorder("a", log, 2));

        runner.run();
        runner.run();

        assertTrue(runner.isFinished());
        assertEquals(List.of("a:init", "a:exec", "a:exec", "a:end"), log);
    }

    @Test
    void runnerIsANoOpOnceFinished() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(new Recorder("a", log, 1));

        runner.run();
        int sizeAfterFinish = log.size();
        runner.run();
        runner.run();

        assertEquals(sizeAfterFinish, log.size(), "must not restart or re-end");
    }

    @Test
    void sequenceRunsCommandsInOrder() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(Commands.sequence(
                new Recorder("a", log, 1),
                new Recorder("b", log, 1)));

        while (!runner.isFinished()) {
            runner.run();
        }

        assertEquals(List.of("a:init", "a:exec", "a:end", "b:init", "b:exec", "b:end"), log);
    }

    @Test
    void sequenceAdvancesPastInstantCommandsWithoutWastingLoops() {
        // A run() that only fires one instant command per loop turns a five-step
        // sequence into a tenth of a second of doing nothing.
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(Commands.sequence(
                Commands.run(() -> log.add("one")),
                Commands.run(() -> log.add("two")),
                Commands.run(() -> log.add("three"))));

        runner.run();

        assertTrue(runner.isFinished(), "all three should complete in one loop");
        assertEquals(List.of("one", "two", "three"), log);
    }

    @Test
    void parallelFinishesOnlyWhenEveryBranchHas() {
        List<String> log = new ArrayList<>();
        Recorder quick = new Recorder("quick", log, 1);
        Recorder slow = new Recorder("slow", log, 3);
        CommandRunner runner = new CommandRunner(Commands.parallel(quick, slow));

        runner.run();
        assertFalse(runner.isFinished(), "should wait for the slow branch");

        while (!runner.isFinished()) {
            runner.run();
        }

        assertTrue(log.contains("quick:end"));
        assertTrue(log.contains("slow:end"));
        assertEquals(3, slow.executions);
    }

    @Test
    void parallelDoesNotKeepExecutingABranchThatFinished() {
        List<String> log = new ArrayList<>();
        Recorder quick = new Recorder("quick", log, 1);
        Recorder slow = new Recorder("slow", log, 4);
        CommandRunner runner = new CommandRunner(Commands.parallel(quick, slow));

        while (!runner.isFinished()) {
            runner.run();
        }

        assertEquals(1, quick.executions, "a finished branch must stop being executed");
    }

    @Test
    void raceFinishesWithTheFirstAndInterruptsTheRest() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(Commands.race(
                new Recorder("quick", log, 1),
                new Recorder("slow", log, 99)));

        while (!runner.isFinished()) {
            runner.run();
        }
        runner.run();

        assertTrue(log.contains("quick:end"), "the winner ends normally");
        assertTrue(log.contains("slow:interrupted"), "the loser is interrupted");
    }

    @Test
    void deadlineEndsWithItsFirstCommandRegardlessOfTheOthers() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(Commands.deadline(
                new Recorder("deadline", log, 2),
                new Recorder("alongside", log, 99)));

        while (!runner.isFinished()) {
            runner.run();
        }

        assertTrue(log.contains("deadline:end"));
        assertTrue(log.contains("alongside:interrupted"));
    }

    @Test
    void cancelInterruptsWhateverIsInFlight() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(Commands.sequence(
                new Recorder("a", log, 99),
                new Recorder("b", log, 1)));

        runner.run();
        runner.cancel();

        assertTrue(runner.isFinished());
        assertTrue(log.contains("a:interrupted"));
        assertFalse(log.contains("b:init"), "a command that never started must not be ended");
    }

    @Test
    void waitSecondsUsesTheSuppliedClock() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        CommandRunner runner = new CommandRunner(Commands.waitSeconds(0.5, clock));

        runner.run();
        assertFalse(runner.isFinished());

        clock.advance(0.6);
        runner.run();
        assertTrue(runner.isFinished());
    }

    @Test
    void waitUntilBlocksOnItsCondition() {
        boolean[] ready = {false};
        CommandRunner runner = new CommandRunner(Commands.waitUntil(() -> ready[0]));

        runner.run();
        assertFalse(runner.isFinished());

        ready[0] = true;
        runner.run();
        assertTrue(runner.isFinished());
    }

    @Test
    void eitherPicksItsBranchWhenItStarts() {
        List<String> log = new ArrayList<>();
        boolean[] flag = {true};
        CommandRunner runner = new CommandRunner(Commands.either(
                () -> flag[0],
                Commands.run(() -> log.add("yes")),
                Commands.run(() -> log.add("no"))));

        runner.run();
        assertEquals(List.of("yes"), log);
    }

    @Test
    void fluentCombinatorsBuildTheSameStructures() {
        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(
                new Recorder("a", log, 1).andThen(new Recorder("b", log, 1)));

        while (!runner.isFinished()) {
            runner.run();
        }

        assertEquals(List.of("a:init", "a:exec", "a:end", "b:init", "b:exec", "b:end"), log);
    }

    // --- integration with the follower ---------------------------------------

    @Test
    void followPathCommandDrivesTheFollowerToCompletion() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants(), clock);

        CommandRunner runner = new CommandRunner(new FollowPathCommand(follower,
                PathChain.of(new BezierLine(new Point(0, 0), new Point(24, 0)))));

        int loops = 0;
        while (!runner.isFinished() && loops++ < 1000) {
            runner.run();
            robot.step(0.02);
        }

        assertTrue(runner.isFinished());
        assertFalse(follower.isBusy());
        assertEquals(24.0, robot.getTruePose().getX(), 1.0);
    }

    @Test
    void aSequenceOfPathsRunsThemBackToBack() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants(), clock);

        List<String> log = new ArrayList<>();
        CommandRunner runner = new CommandRunner(Commands.sequence(
                new FollowPathCommand(follower,
                        PathChain.of(new BezierLine(new Point(0, 0), new Point(24, 0)))),
                Commands.run(() -> log.add("scored")),
                new FollowPathCommand(follower,
                        PathChain.of(new BezierLine(new Point(24, 0), new Point(24, 24))))));

        int loops = 0;
        while (!runner.isFinished() && loops++ < 2000) {
            runner.run();
            robot.step(0.02);
        }

        assertTrue(runner.isFinished(), "sequence did not complete");
        assertEquals(List.of("scored"), log);
        assertEquals(24.0, robot.getTruePose().getX(), 2.0);
        assertEquals(24.0, robot.getTruePose().getY(), 2.0);
    }

    @Test
    void interruptingAPathCommandStopsTheDrivetrain() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);
        PathFollower follower = new PathFollower(robot, robot, constants(), clock);

        // A long path raced against a short timer: the path loses.
        CommandRunner runner = new CommandRunner(Commands.race(
                new FollowPathCommand(follower,
                        PathChain.of(new BezierLine(new Point(0, 0), new Point(96, 0)))),
                Commands.waitSeconds(0.3, clock)));

        int loops = 0;
        while (!runner.isFinished() && loops++ < 1000) {
            runner.run();
            robot.step(0.02);
        }

        assertTrue(runner.isFinished());
        assertFalse(follower.isBusy(), "the follower must be stopped, not left running");
        for (double p : robot.getLastPowers()) {
            assertEquals(0.0, p, 1e-9, "motors must be cut when a path is interrupted");
        }
    }

    private FollowerConstants constants() {
        return FollowerConstants.builder(DistanceUnit.INCH)
                .maxVelocity(30).maxAcceleration(40)
                .kV(1.0 / 60.0)
                .translationalPID(2.0, 0.0, 0.0)
                .headingPID(3.0, 0.0, 0.0)
                .positionTolerance(1.0)
                .headingTolerance(Math.toRadians(3))
                .build();
    }
}
