package com.platapus.pathing.localization;

/**
 * Anything that can report an absolute heading -- in practice, the IMU.
 *
 * <p>Kept as a one-method interface with no FTC types so that the localizer and,
 * in Phase 2, the EKF, can be tested against a scripted heading rather than a
 * real gyro.
 */
@FunctionalInterface
public interface HeadingSource {

    /**
     * Heading in radians CCW, in whatever frame the source was zeroed in. The
     * localizer only ever differences consecutive readings against its own
     * start, so the absolute offset does not matter -- but the value must be
     * continuous, not wrapped, or unwrapping is the caller's job.
     */
    double getHeadingRadians();
}
