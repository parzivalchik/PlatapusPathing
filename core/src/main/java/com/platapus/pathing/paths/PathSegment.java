package com.platapus.pathing.paths;

/** One link of a {@link PathChain}: a curve plus the heading behaviour on it. */
public final class PathSegment {

    private final Path path;
    private final HeadingInterpolator headingInterpolator;
    /** Speed cap for this segment, inches per second, or NaN for "no override". */
    private final double maxVelocity;

    public PathSegment(Path path, HeadingInterpolator headingInterpolator) {
        this(path, headingInterpolator, Double.NaN);
    }

    public PathSegment(Path path, HeadingInterpolator headingInterpolator,
                       double maxVelocity) {
        if (path == null || headingInterpolator == null) {
            throw new IllegalArgumentException("path and headingInterpolator must be non-null");
        }
        if (!Double.isNaN(maxVelocity) && maxVelocity <= 0) {
            throw new IllegalArgumentException(
                    "maxVelocity must be positive or NaN, got " + maxVelocity);
        }
        this.path = path;
        this.headingInterpolator = headingInterpolator;
        this.maxVelocity = maxVelocity;
    }

    /** A segment that faces along its own direction of travel. */
    public PathSegment(Path path) {
        this(path, HeadingInterpolator.tangent());
    }

    public Path getPath() {
        return path;
    }

    public HeadingInterpolator getHeadingInterpolator() {
        return headingInterpolator;
    }

    /** Desired heading at parameter {@code t}. */
    public double headingAt(double t) {
        return headingInterpolator.heading(path, t);
    }

    /** Replaces this segment's heading behaviour, leaving the curve untouched. */
    public PathSegment withHeading(HeadingInterpolator interpolator) {
        return new PathSegment(path, interpolator, maxVelocity);
    }

    /**
     * Caps the speed on this segment, inches per second. NaN removes the cap.
     *
     * <p>The profile still brakes into it ahead of time, so the robot arrives at
     * the segment already slowed rather than braking impossibly hard at the
     * boundary.
     */
    public PathSegment withMaxVelocity(double maxVelocity) {
        return new PathSegment(path, headingInterpolator, maxVelocity);
    }

    /** This segment's speed cap, or NaN if it has none. */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /** Whether this segment overrides the global speed limit. */
    public boolean hasMaxVelocity() {
        return !Double.isNaN(maxVelocity);
    }

    public double length() {
        return path.length();
    }
}
