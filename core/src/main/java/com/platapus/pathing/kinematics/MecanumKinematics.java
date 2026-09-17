package com.platapus.pathing.kinematics;

import com.platapus.pathing.geometry.ChassisSpeeds;
import com.platapus.pathing.units.DistanceUnit;

/**
 * Mecanum kinematics for a standard four-wheel chassis with the rollers in the
 * usual X pattern (top rollers pointing inward toward the robot's centre).
 *
 * <p>Wheel order is front-left, front-right, back-left, back-right throughout.
 *
 * <p>Conventions: +x forward, +y left, omega CCW-positive.
 */
public final class MecanumKinematics implements Kinematics {

    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int BACK_LEFT = 2;
    public static final int BACK_RIGHT = 3;

    /** Left-to-right wheel separation, inches. */
    private final double trackWidth;
    /** Front-to-back wheel separation, inches. */
    private final double wheelBase;
    /** (trackWidth + wheelBase) / 2 -- the rotational lever arm. */
    private final double turnRadius;
    private final double maxWheelVelocity;
    private final double lateralEfficiency;

    private MecanumKinematics(double trackWidth, double wheelBase, double maxWheelVelocity,
                              double lateralEfficiency) {
        if (trackWidth <= 0 || wheelBase <= 0) {
            throw new IllegalArgumentException("trackWidth and wheelBase must be positive");
        }
        if (maxWheelVelocity <= 0) {
            throw new IllegalArgumentException("maxWheelVelocity must be positive");
        }
        if (lateralEfficiency <= 0 || lateralEfficiency > 1.0) {
            throw new IllegalArgumentException(
                    "lateralEfficiency must be in (0, 1]; got " + lateralEfficiency);
        }
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.turnRadius = (trackWidth + wheelBase) / 2.0;
        this.maxWheelVelocity = maxWheelVelocity;
        this.lateralEfficiency = lateralEfficiency;
    }

    /** Builder entry point. All distances are interpreted in {@code unit}. */
    public static Builder builder(DistanceUnit unit) {
        return new Builder(unit);
    }

    public static final class Builder {
        private final DistanceUnit unit;
        private double trackWidth = Double.NaN;
        private double wheelBase = Double.NaN;
        private double maxWheelVelocity = Double.NaN;
        private double lateralEfficiency = 1.0;

        private Builder(DistanceUnit unit) {
            if (unit == null) {
                throw new IllegalArgumentException("unit must be non-null");
            }
            this.unit = unit;
        }

        /** Distance between the left and right wheel centres. */
        public Builder trackWidth(double value) {
            this.trackWidth = unit.toInches(value);
            return this;
        }

        /** Distance between the front and back wheel centres. */
        public Builder wheelBase(double value) {
            this.wheelBase = unit.toInches(value);
            return this;
        }

        /** Top tangential speed of one wheel, per second, in this builder's unit. */
        public Builder maxWheelVelocity(double value) {
            this.maxWheelVelocity = unit.toInches(value);
            return this;
        }

        /**
         * How much of the commanded lateral speed the chassis actually achieves,
         * in (0, 1]. Mecanum strafing loses more to roller scrub than driving
         * forward does; 0.8 to 0.9 is typical once measured. Leave at 1.0 until
         * you have measured it, so the value is honest rather than guessed.
         */
        public Builder lateralEfficiency(double value) {
            this.lateralEfficiency = value;
            return this;
        }

        public MecanumKinematics build() {
            if (Double.isNaN(trackWidth) || Double.isNaN(wheelBase)) {
                throw new IllegalStateException("trackWidth and wheelBase are required");
            }
            if (Double.isNaN(maxWheelVelocity)) {
                throw new IllegalStateException("maxWheelVelocity is required");
            }
            return new MecanumKinematics(trackWidth, wheelBase, maxWheelVelocity,
                    lateralEfficiency);
        }
    }

    @Override
    public int getWheelCount() {
        return 4;
    }

    @Override
    public double[] toWheelVelocities(ChassisSpeeds speeds) {
        double vx = speeds.vx;
        // Ask for more lateral wheel speed than the naive model says, to make up
        // for what roller scrub eats. At efficiency 1.0 this is a no-op.
        double vy = speeds.vy / lateralEfficiency;
        double rot = speeds.omega * turnRadius;

        double[] out = new double[4];
        out[FRONT_LEFT] = vx - vy - rot;
        out[FRONT_RIGHT] = vx + vy + rot;
        out[BACK_LEFT] = vx + vy - rot;
        out[BACK_RIGHT] = vx - vy + rot;
        return out;
    }

    @Override
    public ChassisSpeeds toChassisSpeeds(double[] w) {
        if (w.length != 4) {
            throw new IllegalArgumentException("Mecanum expects 4 wheel values, got " + w.length);
        }
        double vx = (w[FRONT_LEFT] + w[FRONT_RIGHT] + w[BACK_LEFT] + w[BACK_RIGHT]) / 4.0;
        double vy = (-w[FRONT_LEFT] + w[FRONT_RIGHT] + w[BACK_LEFT] - w[BACK_RIGHT]) / 4.0
                * lateralEfficiency;
        double omega = (-w[FRONT_LEFT] + w[FRONT_RIGHT] - w[BACK_LEFT] + w[BACK_RIGHT])
                / (4.0 * turnRadius);
        return new ChassisSpeeds(vx, vy, omega);
    }

    @Override
    public boolean canStrafe() {
        return true;
    }

    @Override
    public double getMaxLinearVelocity() {
        return maxWheelVelocity;
    }

    @Override
    public double getMaxWheelVelocity() {
        return maxWheelVelocity;
    }

    /**
     * The fastest the chassis can rotate in place, radians per second: all four
     * wheels at top speed, all contributing to yaw.
     */
    public double getMaxAngularVelocity() {
        return maxWheelVelocity / turnRadius;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public double getLateralEfficiency() {
        return lateralEfficiency;
    }
}
