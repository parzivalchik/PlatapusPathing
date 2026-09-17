package com.platapus.pathing;

import com.platapus.pathing.control.FollowerConstants;
import com.platapus.pathing.follower.PathFollower;
import com.platapus.pathing.geometry.Pose2d;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.paths.BezierLine;
import com.platapus.pathing.paths.PathChain;
import com.platapus.pathing.paths.Point;
import com.platapus.pathing.sim.SimulatedRobot;
import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.FileTuningStore;
import com.platapus.pathing.tuning.TunableFeedforward;
import com.platapus.pathing.tuning.TuningSession;
import com.platapus.pathing.tuning.VoltageSource;
import com.platapus.pathing.units.DistanceUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TuningIntegrationTest {

    // --- TunableFeedforward: the mid-path safety property ---------------------

    @Test
    void proposalsDoNotTakeEffectUntilCommitted() {
        FeedforwardGains initial = new FeedforwardGains(0.05, 0.017, 0.001);
        TunableFeedforward ff = new TunableFeedforward(initial);

        assertTrue(ff.propose(new FeedforwardGains(0.09, 0.021, 0.002)));
        assertTrue(ff.hasPending());
        assertEquals(initial, ff.get(), "a proposal must not change the active gains");

        assertTrue(ff.commitPending());
        assertEquals(0.021, ff.get().kV, 1e-12);
        assertFalse(ff.hasPending());
    }

    @Test
    void implausibleProposalsAreRefused() {
        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.05, 0.017, 0.001));

        assertFalse(ff.propose(new FeedforwardGains(0.05, Double.NaN, 0.001)));
        assertFalse(ff.propose(new FeedforwardGains(0.05, -0.017, 0.001)), "negative kV");
        assertFalse(ff.propose(new FeedforwardGains(0.05, 5.0, 0.001)), "absurd kV");
        assertFalse(ff.propose(null));
        assertFalse(ff.hasPending());
    }

    @Test
    void aProposalThatBarelyDiffersIsNotWorthAdopting() {
        FeedforwardGains initial = new FeedforwardGains(0.05, 0.017, 0.001);
        TunableFeedforward ff = new TunableFeedforward(initial);

        assertFalse(ff.propose(new FeedforwardGains(0.0501, 0.01701, 0.001)),
                "a negligible change is churn, not an improvement");
        assertFalse(ff.hasPending());
    }

    @Test
    void theFollowerAdoptsGainsOnlyWhenAPathStarts() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);

        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.0, 1.0 / 60, 0.0));
        PathFollower follower = new PathFollower(robot, robot, constants(), clock, ff);

        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(48, 0))));

        // Propose a very different model while a path is in flight.
        FeedforwardGains mid = new FeedforwardGains(0.10, 0.030, 0.002);
        assertTrue(ff.propose(mid));

        for (int i = 0; i < 40; i++) {
            follower.update();
            robot.step(0.02);
            assertEquals(1.0 / 60, ff.get().kV, 1e-12,
                    "gains changed mid-path at loop " + i);
        }

        // Only the next path picks it up.
        follower.followPath(PathChain.of(new BezierLine(new Point(0, 0), new Point(10, 0))));
        assertEquals(mid, ff.get());
        assertEquals(1, ff.getCommitCount());
    }

    @Test
    void loadingSavedGainsAtInitBypassesThePendingQueue() {
        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.0, 0.020, 0.0));
        FeedforwardGains saved = new FeedforwardGains(0.07, 0.0165, 0.0011);

        ff.setActiveImmediately(saved);

        assertEquals(saved, ff.get());
        assertFalse(ff.hasPending());
    }

    // --- persistence ----------------------------------------------------------

    @Test
    void gainsSurviveASaveAndReload(@TempDir Path dir) {
        FileTuningStore store = new FileTuningStore(dir.resolve("gains.properties").toFile());
        assertNull(store.load(), "nothing saved yet");

        FeedforwardGains gains = new FeedforwardGains(0.081, 0.01654, 0.00121);
        assertTrue(store.save(gains));

        FeedforwardGains loaded = store.load();
        assertNotNull(loaded);
        assertEquals(gains.kS, loaded.kS, 1e-12);
        assertEquals(gains.kV, loaded.kV, 1e-12);
        assertEquals(gains.kA, loaded.kA, 1e-12);
    }

    @Test
    void savingCreatesMissingDirectories(@TempDir Path dir) {
        FileTuningStore store = new FileTuningStore(
                dir.resolve("nested/deeper/gains.properties").toFile());
        assertTrue(store.save(new FeedforwardGains(0.05, 0.017, 0.001)));
        assertNotNull(store.load());
    }

    @Test
    void aCorruptFileIsRefusedRatherThanLoadedAsNonsense(@TempDir Path dir) throws IOException {
        File file = dir.resolve("gains.properties").toFile();
        Files.writeString(file.toPath(), "kS=banana\nkV=oops\nkA=0.001\n");

        assertNull(new FileTuningStore(file).load(),
                "driving on a kV of NaN is not recoverable; refusing to load is");
    }

    @Test
    void anImplausibleSavedFileIsRefused(@TempDir Path dir) throws IOException {
        File file = dir.resolve("gains.properties").toFile();
        Files.writeString(file.toPath(), "kS=0.05\nkV=-0.017\nkA=0.001\n");
        assertNull(new FileTuningStore(file).load());
    }

    @Test
    void implausibleGainsAreNotSaved(@TempDir Path dir) {
        FileTuningStore store = new FileTuningStore(dir.resolve("gains.properties").toFile());
        assertFalse(store.save(new FeedforwardGains(0.05, Double.NaN, 0.001)));
        assertFalse(store.save(null));
        assertNull(store.load());
    }

    @Test
    void clearRemovesTheSavedFit(@TempDir Path dir) {
        FileTuningStore store = new FileTuningStore(dir.resolve("gains.properties").toFile());
        store.save(new FeedforwardGains(0.05, 0.017, 0.001));
        assertTrue(store.clear());
        assertNull(store.load());
    }

    // --- the whole loop, on a simulated robot --------------------------------

    /**
     * A robot whose real feedforward differs from what the follower was told,
     * so there is something for the tuner to actually discover.
     */
    @Test
    void aTuningSessionFitsTheRobotFromOrdinaryDriving(@TempDir Path dir) {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);

        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.0, 0.030, 0.0));
        PathFollower follower = new PathFollower(robot, robot, constants(), clock, ff);

        FileTuningStore store = new FileTuningStore(dir.resolve("gains.properties").toFile());
        TuningSession session = TuningSession.builder(
                        robot, VoltageSource.constant(12.6), clock, ff)
                .minSamplesBeforeProposing(200)
                .minConfidenceBeforeProposing(0.0)
                .store(store)
                .build();

        // Drive several paths, tuning throughout.
        for (int p = 0; p < 4; p++) {
            follower.followPath(PathChain.of(
                    new BezierLine(new Point(0, 0), new Point(48, 0))));
            int loops = 0;
            while (follower.isBusy() && loops++ < 1000) {
                follower.update();
                session.update(follower.getLastPowers());
                robot.step(0.02);
            }
            robot.setPose(Pose2d.ZERO);
        }

        assertTrue(session.getTuner().getSampleCount() > 200,
                "should have collected samples, got " + session.getTuner().getSampleCount());
        assertTrue(session.hasProposed(), "a fit should have been offered");
        assertTrue(ff.getCommitCount() > 0, "and adopted at a path boundary");

        assertTrue(session.persist(), "should save what the follower actually used");
        assertNotNull(store.load());
    }

    @Test
    void restoreLoadsSavedGainsIntoTheFeedforward(@TempDir Path dir) {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);

        FileTuningStore store = new FileTuningStore(dir.resolve("gains.properties").toFile());
        FeedforwardGains saved = new FeedforwardGains(0.077, 0.0169, 0.0013);
        store.save(saved);

        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.0, 0.030, 0.0));
        TuningSession session = TuningSession.builder(
                        robot, VoltageSource.constant(12.5), clock, ff)
                .store(store)
                .build();

        assertTrue(session.restore());
        assertEquals(saved.kV, ff.get().kV, 1e-12,
                "a second OpMode run should start where the first left off");
    }

    @Test
    void batterySagChangesTheNormalisedCommandNotJustTheDuty() {
        // The reason samples are normalised by battery voltage: the same duty is
        // a different applied voltage at 11 V than at 13 V, and a model fitted
        // against raw duty silently re-fits itself as the battery drains.
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();

        double[] battery = {13.0};
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);
        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.0, 0.017, 0.0));
        TuningSession session = TuningSession.builder(
                robot, () -> battery[0], clock, ff).build();

        robot.setWheelPowers(new double[]{0.5, 0.5, 0.5, 0.5});
        clock.advance(0.02);
        robot.step(0.02);
        session.update(new double[]{0.5, 0.5, 0.5, 0.5});

        long afterFirst = session.getTuner().getSampleCount();

        battery[0] = 11.0;
        clock.advance(0.02);
        robot.step(0.02);
        session.update(new double[]{0.5, 0.5, 0.5, 0.5});

        assertTrue(session.getTuner().getSampleCount() > afterFirst,
                "samples should still be accepted at a lower battery voltage");
    }

    @Test
    void anImplausibleBatteryReadingIsIgnored() {
        SimulatedRobot.ManualClock clock = new SimulatedRobot.ManualClock();
        MecanumKinematics kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(15).wheelBase(13).maxWheelVelocity(60).build();
        SimulatedRobot robot = new SimulatedRobot(kinematics, clock, Pose2d.ZERO);

        TunableFeedforward ff = new TunableFeedforward(new FeedforwardGains(0.0, 0.017, 0.0));
        TuningSession session = TuningSession.builder(
                robot, VoltageSource.constant(0.0), clock, ff).build();

        robot.setWheelPowers(new double[]{0.5, 0.5, 0.5, 0.5});
        for (int i = 0; i < 20; i++) {
            clock.advance(0.02);
            robot.step(0.02);
            session.update(new double[]{0.5, 0.5, 0.5, 0.5});
        }

        assertEquals(0, session.getTuner().getSampleCount(),
                "a nonsense battery reading would poison every sample");
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
