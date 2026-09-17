package com.platapus.pathing.command;

import com.platapus.pathing.control.Clock;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

/** Factories and combinators for {@link Command}. */
public final class Commands {

    private Commands() {
    }

    /** Does nothing and finishes immediately. */
    public static Command none() {
        return new Command() {
            @Override public void execute() { }
            @Override public boolean isFinished() { return true; }
        };
    }

    /** Runs {@code action} once, then finishes. */
    public static Command run(Runnable action) {
        requireNonNull(action, "action");
        return new Command() {
            @Override
            public void initialize() {
                action.run();
            }

            @Override public void execute() { }
            @Override public boolean isFinished() { return true; }
        };
    }

    /** Runs {@code action} every loop until {@code until} returns true. */
    public static Command runUntil(Runnable action, BooleanSupplier until) {
        requireNonNull(action, "action");
        requireNonNull(until, "until");
        return new Command() {
            @Override public void execute() { action.run(); }
            @Override public boolean isFinished() { return until.getAsBoolean(); }
        };
    }

    /** Finishes once {@code condition} becomes true. Does nothing meanwhile. */
    public static Command waitUntil(BooleanSupplier condition) {
        requireNonNull(condition, "condition");
        return new Command() {
            @Override public void execute() { }
            @Override public boolean isFinished() { return condition.getAsBoolean(); }
        };
    }

    /** Finishes {@code seconds} after it starts. */
    public static Command waitSeconds(double seconds, Clock clock) {
        requireNonNull(clock, "clock");
        return new Command() {
            private double deadline;

            @Override
            public void initialize() {
                deadline = clock.seconds() + seconds;
            }

            @Override public void execute() { }
            @Override public boolean isFinished() { return clock.seconds() >= deadline; }
        };
    }

    /** Runs the given commands one after another. */
    public static Command sequence(Command... commands) {
        return new SequentialCommand(Arrays.asList(commands));
    }

    /** Runs the given commands one after another. */
    public static Command sequence(List<Command> commands) {
        return new SequentialCommand(commands);
    }

    /** Runs all the given commands at once, finishing when every one has. */
    public static Command parallel(Command... commands) {
        return new ParallelCommand(Arrays.asList(commands), false);
    }

    /**
     * Runs all the given commands at once, finishing as soon as any one does.
     * The rest are interrupted.
     */
    public static Command race(Command... commands) {
        return new ParallelCommand(Arrays.asList(commands), true);
    }

    /**
     * Runs {@code deadline} alongside {@code others}, finishing when {@code
     * deadline} does. Useful for "keep the intake running until we get there".
     */
    public static Command deadline(Command deadline, Command... others) {
        requireNonNull(deadline, "deadline");
        List<Command> all = new ArrayList<>();
        all.add(deadline);
        all.addAll(Arrays.asList(others));
        return new ParallelCommand(all, true, 0);
    }

    /** Picks between two commands when it starts, based on {@code condition}. */
    public static Command either(BooleanSupplier condition, Command ifTrue, Command ifFalse) {
        requireNonNull(condition, "condition");
        requireNonNull(ifTrue, "ifTrue");
        requireNonNull(ifFalse, "ifFalse");
        return new Command() {
            private Command chosen;

            @Override
            public void initialize() {
                chosen = condition.getAsBoolean() ? ifTrue : ifFalse;
                chosen.initialize();
            }

            @Override public void execute() { chosen.execute(); }
            @Override public boolean isFinished() { return chosen.isFinished(); }
            @Override public void end(boolean interrupted) { chosen.end(interrupted); }
        };
    }

    static void requireNonNull(Object o, String name) {
        if (o == null) {
            throw new IllegalArgumentException(name + " must be non-null");
        }
    }

    // --- implementations -----------------------------------------------------

    private static final class SequentialCommand implements Command {
        private final List<Command> commands;
        private int index;
        private boolean started;

        SequentialCommand(List<Command> commands) {
            requireNonNull(commands, "commands");
            for (Command c : commands) {
                requireNonNull(c, "command in sequence");
            }
            this.commands = new ArrayList<>(commands);
        }

        @Override
        public void initialize() {
            index = 0;
            started = false;
        }

        @Override
        public void execute() {
            if (index >= commands.size()) {
                return;
            }

            Command current = commands.get(index);
            if (!started) {
                current.initialize();
                started = true;
            }
            current.execute();

            // Advance as far as possible this loop: a command that finishes
            // immediately should not cost a whole cycle of doing nothing.
            while (index < commands.size() && commands.get(index).isFinished()) {
                commands.get(index).end(false);
                index++;
                started = false;
                if (index < commands.size()) {
                    commands.get(index).initialize();
                    started = true;
                    commands.get(index).execute();
                }
            }
        }

        @Override
        public boolean isFinished() {
            return index >= commands.size();
        }

        @Override
        public void end(boolean interrupted) {
            // Only the command actually in flight needs ending; the ones before
            // it already ended, and the ones after never started.
            if (interrupted && started && index < commands.size()) {
                commands.get(index).end(true);
            }
        }
    }

    private static final class ParallelCommand implements Command {
        private final List<Command> commands;
        private final boolean finishOnFirst;
        /** When >= 0, only this command's completion ends the group. */
        private final int deadlineIndex;
        private boolean[] running;

        ParallelCommand(List<Command> commands, boolean finishOnFirst) {
            this(commands, finishOnFirst, -1);
        }

        ParallelCommand(List<Command> commands, boolean finishOnFirst, int deadlineIndex) {
            requireNonNull(commands, "commands");
            for (Command c : commands) {
                requireNonNull(c, "command in group");
            }
            this.commands = new ArrayList<>(commands);
            this.finishOnFirst = finishOnFirst;
            this.deadlineIndex = deadlineIndex;
        }

        @Override
        public void initialize() {
            running = new boolean[commands.size()];
            for (int i = 0; i < commands.size(); i++) {
                commands.get(i).initialize();
                running[i] = true;
            }
        }

        @Override
        public void execute() {
            for (int i = 0; i < commands.size(); i++) {
                if (!running[i]) {
                    continue;
                }
                commands.get(i).execute();
                if (commands.get(i).isFinished()) {
                    commands.get(i).end(false);
                    running[i] = false;
                }
            }
        }

        @Override
        public boolean isFinished() {
            if (deadlineIndex >= 0) {
                return !running[deadlineIndex];
            }
            if (finishOnFirst) {
                for (boolean r : running) {
                    if (!r) {
                        return true;
                    }
                }
                return false;
            }
            for (boolean r : running) {
                if (r) {
                    return false;
                }
            }
            return true;
        }

        @Override
        public void end(boolean interrupted) {
            // Whatever is still running when the group ends was cut short,
            // whether the group itself was interrupted or simply won its race.
            for (int i = 0; i < commands.size(); i++) {
                if (running[i]) {
                    commands.get(i).end(true);
                    running[i] = false;
                }
            }
        }
    }
}
