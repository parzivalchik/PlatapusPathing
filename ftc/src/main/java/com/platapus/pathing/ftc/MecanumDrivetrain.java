package com.platapus.pathing.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.platapus.pathing.drive.Drivetrain;
import com.platapus.pathing.kinematics.Kinematics;
import com.platapus.pathing.kinematics.MecanumKinematics;
import com.platapus.pathing.units.DistanceUnit;

/**
 * The FTC implementation of {@link Drivetrain}: four {@code DcMotorEx} and the
 * arithmetic to turn encoder ticks into inches.
 *
 * <p>Everything else -- paths, profiles, control, kinematics -- lives in
 * {@code :core} and knows nothing about the FTC SDK. This class is the entire
 * hardware surface of the library for a mecanum robot.
 */
public final class MecanumDrivetrain implements Drivetrain {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx[] motors;

    private final MecanumKinematics kinematics;
    /** Inches of wheel travel per encoder tick. */
    private final double inchesPerTick;

    private MecanumDrivetrain(Builder b) {
        this.frontLeft = b.frontLeft;
        this.frontRight = b.frontRight;
        this.backLeft = b.backLeft;
        this.backRight = b.backRight;
        this.motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        double wheelCircumference = 2.0 * Math.PI * b.wheelRadius;
        this.inchesPerTick = wheelCircumference * b.gearRatio / b.ticksPerRevolution;

        double maxWheelVelocity = b.maxMotorRpm / 60.0 * wheelCircumference * b.gearRatio;
        this.kinematics = MecanumKinematics.builder(DistanceUnit.INCH)
                .trackWidth(b.trackWidth)
                .wheelBase(b.wheelBase)
                .maxWheelVelocity(maxWheelVelocity)
                .lateralEfficiency(b.lateralEfficiency)
                .build();

        for (DcMotorEx motor : motors) {
            // Coasting between loops makes the robot's response to a power
            // command depend on how long the previous loop took. Brake mode
            // makes it consistent, which the feedforward model assumes.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // The follower closes its own loop on pose. Letting the motor
            // controller close a second loop on velocity underneath it means two
            // controllers fighting over the same actuator.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Builder. All distances are interpreted in {@code unit}; angles, as always,
     * are radians.
     */
    public static Builder builder(DistanceUnit unit) {
        return new Builder(unit);
    }

    public static final class Builder {
        private final DistanceUnit unit;

        private DcMotorEx frontLeft;
        private DcMotorEx frontRight;
        private DcMotorEx backLeft;
        private DcMotorEx backRight;

        private double trackWidth = Double.NaN;
        private double wheelBase = Double.NaN;
        private double wheelRadius = Double.NaN;
        private double gearRatio = 1.0;
        private double ticksPerRevolution = Double.NaN;
        private double maxMotorRpm = Double.NaN;
        private double lateralEfficiency = 1.0;

        private Builder(DistanceUnit unit) {
            this.unit = unit;
        }

        /** Looks the four motors up by name in the configuration. */
        public Builder motors(HardwareMap hardwareMap, String frontLeftName,
                              String frontRightName, String backLeftName,
                              String backRightName) {
            this.frontLeft = hardwareMap.get(DcMotorEx.class, frontLeftName);
            this.frontRight = hardwareMap.get(DcMotorEx.class, frontRightName);
            this.backLeft = hardwareMap.get(DcMotorEx.class, backLeftName);
            this.backRight = hardwareMap.get(DcMotorEx.class, backRightName);
            return this;
        }

        /** Supplies already-resolved motors. */
        public Builder motors(DcMotorEx frontLeft, DcMotorEx frontRight,
                              DcMotorEx backLeft, DcMotorEx backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
            return this;
        }

        /**
         * Reverses the given motors.
         *
         * <p>On almost every mecanum build one side is mounted mirrored and must
         * be reversed, otherwise "forward" makes the robot spin. Check this
         * first when a brand new robot does something baffling.
         *
         * <p>Must be called <b>after</b> {@link #motors}. It acts on the motor
         * objects directly, so there is nothing to reverse before they exist --
         * and a silent no-op here would leave a robot driving wrong with no
         * indication why, which is exactly the failure this method is for.
         *
         * @throws IllegalStateException if the motors have not been supplied yet
         */
        public Builder reverse(boolean frontLeft, boolean frontRight,
                               boolean backLeft, boolean backRight) {
            requireMotors("reverse");
            applyDirection(this.frontLeft, frontLeft);
            applyDirection(this.frontRight, frontRight);
            applyDirection(this.backLeft, backLeft);
            applyDirection(this.backRight, backRight);
            return this;
        }

        private void requireMotors(String what) {
            if (this.frontLeft == null || this.frontRight == null
                    || this.backLeft == null || this.backRight == null) {
                throw new IllegalStateException(
                        what + "() was called before motors(); call motors() first, "
                                + "or the setting is silently dropped");
            }
        }

        private static void applyDirection(DcMotorEx motor, boolean reversed) {
            motor.setDirection(reversed
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
        }

        /** Left-to-right wheel separation. */
        public Builder trackWidth(double value) {
            this.trackWidth = unit.toInches(value);
            return this;
        }

        /** Front-to-back wheel separation. */
        public Builder wheelBase(double value) {
            this.wheelBase = unit.toInches(value);
            return this;
        }

        /** Wheel radius. A 96 mm goBILDA mecanum wheel is 48 mm. */
        public Builder wheelRadius(double value) {
            this.wheelRadius = unit.toInches(value);
            return this;
        }

        /**
         * Output revolutions per motor revolution. Leave at 1 when the wheel is
         * mounted straight onto the motor's own gearbox output.
         */
        public Builder gearRatio(double ratio) {
            this.gearRatio = ratio;
            return this;
        }

        /** Encoder ticks per revolution of the gearbox output shaft. */
        public Builder ticksPerRevolution(double ticks) {
            this.ticksPerRevolution = ticks;
            return this;
        }

        /** Free speed of the gearbox output, RPM. From the motor's datasheet. */
        public Builder maxMotorRpm(double rpm) {
            this.maxMotorRpm = rpm;
            return this;
        }

        /** See {@link MecanumKinematics.Builder#lateralEfficiency(double)}. */
        public Builder lateralEfficiency(double efficiency) {
            this.lateralEfficiency = efficiency;
            return this;
        }

        public MecanumDrivetrain build() {
            if (frontLeft == null || frontRight == null || backLeft == null
                    || backRight == null) {
                throw new IllegalStateException("all four motors must be supplied");
            }
            if (Double.isNaN(trackWidth) || Double.isNaN(wheelBase)
                    || Double.isNaN(wheelRadius) || Double.isNaN(ticksPerRevolution)
                    || Double.isNaN(maxMotorRpm)) {
                throw new IllegalStateException("trackWidth, wheelBase, wheelRadius, "
                        + "ticksPerRevolution and maxMotorRpm are all required");
            }
            return new MecanumDrivetrain(this);
        }
    }

    @Override
    public Kinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void setWheelPowers(double[] powers) {
        if (powers.length != 4) {
            throw new IllegalArgumentException("expected 4 powers, got " + powers.length);
        }
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    @Override
    public double[] getWheelPositions() {
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) {
            out[i] = motors[i].getCurrentPosition() * inchesPerTick;
        }
        return out;
    }

    @Override
    public double[] getWheelVelocities() {
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) {
            out[i] = motors[i].getVelocity() * inchesPerTick;
        }
        return out;
    }

    /**
     * Present current draw of each motor, amps, in wheel order.
     *
     * <p>What {@link com.platapus.pathing.tuning.StallDetector} compares
     * against the feedforward model's prediction.
     */
    public double[] getMotorCurrents() {
        double[] out = new double[4];
        for (int i = 0; i < 4; i++) {
            out[i] = motors[i].getCurrent(CurrentUnit.AMPS);
        }
        return out;
    }

    /** Zeroes all four encoders and returns to open-loop power control. */
    public void resetEncoders() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** The motors, in kinematics order. Phase 3's stall detection reads current here. */
    public DcMotorEx[] getMotors() {
        return motors.clone();
    }
}
