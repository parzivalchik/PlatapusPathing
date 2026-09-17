package com.platapus.pathing.paths;

import com.platapus.pathing.units.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Fluent builder for a {@link PathChain}.
 *
 * <p>The shape is deliberately close to Pedro Pathing's, so an auto ported from
 * Pedro reads almost unchanged:
 *
 * <pre>{@code
 * PathChain chain = new PathBuilder()
 *         .addPath(new BezierLine(new Point(0, 0), new Point(24, 0)))
 *         .setLinearHeadingInterpolation(0, Math.toRadians(90))
 *         .addPath(new BezierCurve(new Point(24, 0), new Point(36, 12), new Point(48, 24)))
 *         .setConstantHeadingInterpolation(Math.toRadians(90))
 *         .build();
 * }</pre>
 *
 * <p>Each {@code set...HeadingInterpolation} call applies to the path most
 * recently added. A segment with no heading call defaults to following its own
 * tangent.
 *
 * <p>To work in centimetres, set the unit once up front and every bare
 * coordinate passed to the builder's own {@code line}/{@code curve} helpers is
 * interpreted in it.
 */
public final class PathBuilder {

    private final List<PathSegment> segments = new ArrayList<>();
    private DistanceUnit unit = DistanceUnit.INCH;

    /**
     * Sets the unit for coordinates passed to this builder's {@link #line} and
     * {@link #curve} helpers. Does not affect {@link Point} instances you
     * construct yourself -- those carry their own unit at construction.
     */
    public PathBuilder setUnit(DistanceUnit unit) {
        if (unit == null) {
            throw new IllegalArgumentException("unit must be non-null");
        }
        this.unit = unit;
        return this;
    }

    /** Appends a path, defaulting to tangent heading. */
    public PathBuilder addPath(Path path) {
        segments.add(new PathSegment(path));
        return this;
    }

    /** Appends a straight segment, in this builder's unit. */
    public PathBuilder line(double x1, double y1, double x2, double y2) {
        return addPath(new BezierLine(Point.of(x1, y1, unit), Point.of(x2, y2, unit)));
    }

    /**
     * Appends a Bezier through the given coordinates, in this builder's unit.
     * Pass x/y pairs: start, control(s), end.
     */
    public PathBuilder curve(double... coordinates) {
        if (coordinates.length < 6 || coordinates.length % 2 != 0) {
            throw new IllegalArgumentException(
                    "curve() needs an even number of coordinates, at least 6 (3 points); got "
                            + coordinates.length);
        }
        List<Point> points = new ArrayList<>(coordinates.length / 2);
        for (int i = 0; i < coordinates.length; i += 2) {
            points.add(Point.of(coordinates[i], coordinates[i + 1], unit));
        }
        return addPath(new BezierCurve(points));
    }

    /** Holds {@code headingRad} across the most recently added path. */
    public PathBuilder setConstantHeadingInterpolation(double headingRad) {
        return setHeadingInterpolation(HeadingInterpolator.constant(headingRad));
    }

    /** Sweeps heading across the most recently added path, the short way around. */
    public PathBuilder setLinearHeadingInterpolation(double startRad, double endRad) {
        return setHeadingInterpolation(HeadingInterpolator.linear(startRad, endRad));
    }

    /** Sweeps heading the long way around -- through the reflex angle. */
    public PathBuilder setReversedHeadingInterpolation(double startRad, double endRad) {
        return setHeadingInterpolation(HeadingInterpolator.linearReversed(startRad, endRad));
    }

    /** Faces along the direction of travel. This is already the default. */
    public PathBuilder setTangentHeadingInterpolation() {
        return setHeadingInterpolation(HeadingInterpolator.tangent());
    }

    /** Faces backwards along the direction of travel. */
    public PathBuilder setReverseTangentHeadingInterpolation() {
        return setHeadingInterpolation(HeadingInterpolator.reverseTangent());
    }

    /**
     * Caps the speed on the most recently added path, in this builder's unit per
     * second.
     *
     * <p>Use it for the parts of an auto that need care -- lining up on a scoring
     * position, threading a gap -- without slowing down the whole run. The
     * profile brakes into the capped segment ahead of the boundary, so the robot
     * is already at the lower speed when it arrives.
     */
    public PathBuilder setMaxVelocity(double maxVelocity) {
        requireSegment("setMaxVelocity");
        int last = segments.size() - 1;
        segments.set(last, segments.get(last).withMaxVelocity(unit.toInches(maxVelocity)));
        return this;
    }

    /** Applies an arbitrary interpolator to the most recently added path. */
    public PathBuilder setHeadingInterpolation(HeadingInterpolator interpolator) {
        requireSegment("setHeadingInterpolation");
        int last = segments.size() - 1;
        segments.set(last, segments.get(last).withHeading(interpolator));
        return this;
    }

    public PathChain build() {
        requireSegment("build");
        return new PathChain(segments);
    }

    private void requireSegment(String what) {
        if (segments.isEmpty()) {
            throw new IllegalStateException(
                    what + "() was called before any path was added; call addPath() first");
        }
    }
}
