package com.platapus.pathing.units;

/**
 * Linear distance unit.
 *
 * <p>PlatapusPathing stores every distance internally in <b>inches</b>. That is an
 * implementation detail: you never have to work in inches. A unit is attached at
 * the boundary -- when you build a path, configure a drivetrain, or read a pose
 * back out -- and converted to the canonical unit immediately. Nothing inside the
 * follower, the profile, or the kinematics ever carries a unit tag around, so
 * there is no way for a centimetre to leak into a computation expecting inches.
 *
 * <p>Angles are always radians, everywhere, with no unit switch. See
 * {@code Conventions} in the README.
 */
public enum DistanceUnit {

    INCH(1.0),
    CM(1.0 / 2.54),
    MM(1.0 / 25.4),
    METER(1.0 / 0.0254);

    /** How many canonical inches one of this unit is worth. */
    private final double inchesPerUnit;

    DistanceUnit(double inchesPerUnit) {
        this.inchesPerUnit = inchesPerUnit;
    }

    /** Converts {@code value}, expressed in this unit, to canonical inches. */
    public double toInches(double value) {
        return value * inchesPerUnit;
    }

    /** Converts {@code inches} (canonical) into this unit. */
    public double fromInches(double inches) {
        return inches / inchesPerUnit;
    }

    /** Converts {@code value} from this unit into {@code target}. */
    public double convertTo(double value, DistanceUnit target) {
        return target.fromInches(toInches(value));
    }
}
