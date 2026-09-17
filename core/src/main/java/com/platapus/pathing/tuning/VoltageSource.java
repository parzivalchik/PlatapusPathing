package com.platapus.pathing.tuning;

/**
 * Reports the battery voltage.
 *
 * <p>A one-method interface with no FTC types so the tuner and the stall
 * detector can be tested against a scripted battery -- including one that sags,
 * which is the whole reason the reading matters.
 */
@FunctionalInterface
public interface VoltageSource {

    /** Present battery voltage, volts. */
    double getVoltage();

    /** A fixed voltage. For tests, and for hardware that cannot report one. */
    static VoltageSource constant(double volts) {
        return () -> volts;
    }
}
