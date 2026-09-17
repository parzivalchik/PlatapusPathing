package com.platapus.pathing.paths;

import com.platapus.pathing.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * An ordered run of {@link PathSegment}s driven as one continuous motion.
 *
 * <p>The chain presents itself to the follower as a single curve parameterised by
 * arc length from 0 to {@link #length()}, hiding the segment boundaries. That is
 * what lets one motion profile span the whole chain instead of stopping and
 * restarting at every junction.
 *
 * <p>Segments are not required to be continuous. A gap or a corner between two
 * segments is legal and simply means the robot has to get there; it is worth
 * knowing that a sharp corner will be profiled as if it were smooth, so the
 * follower will cut it somewhat. Build a proper curve if that matters.
 */
public final class PathChain {

    private final List<PathSegment> segments;
    /** startArcLength[i] = distance from chain start to segment i's start. */
    private final double[] startArcLength;
    private final double totalLength;

    public PathChain(List<PathSegment> segments) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException("A PathChain needs at least one segment");
        }
        this.segments = Collections.unmodifiableList(new ArrayList<>(segments));

        this.startArcLength = new double[this.segments.size()];
        double running = 0.0;
        for (int i = 0; i < this.segments.size(); i++) {
            startArcLength[i] = running;
            running += this.segments.get(i).length();
        }
        this.totalLength = running;
    }

    public static PathChain of(PathSegment... segments) {
        return new PathChain(java.util.Arrays.asList(segments));
    }

    /** A one-segment chain, facing along the path. */
    public static PathChain of(Path path) {
        return new PathChain(Collections.singletonList(new PathSegment(path)));
    }

    public List<PathSegment> getSegments() {
        return segments;
    }

    public int size() {
        return segments.size();
    }

    public PathSegment getSegment(int index) {
        return segments.get(index);
    }

    /** Total arc length of the whole chain, inches. */
    public double length() {
        return totalLength;
    }

    /** Arc length at which segment {@code index} begins. */
    public double segmentStartArcLength(int index) {
        return startArcLength[index];
    }

    /** Resolves the chain at arc length {@code s} inches from the start. */
    public PathState stateAtArcLength(double s) {
        double clamped = Math.max(0.0, Math.min(totalLength, s));

        int index = segments.size() - 1;
        for (int i = 0; i < segments.size(); i++) {
            double end = startArcLength[i] + segments.get(i).length();
            if (clamped <= end) {
                index = i;
                break;
            }
        }

        PathSegment segment = segments.get(index);
        double local = clamped - startArcLength[index];
        double t = segment.getPath().tAtArcLength(local);
        return stateAt(index, t, clamped);
    }

    /** Resolves the chain at parameter {@code t} within segment {@code index}. */
    public PathState stateAtSegment(int index, double t) {
        double clampedT = Math.max(0.0, Math.min(1.0, t));
        double s = startArcLength[index] + segments.get(index).getPath().arcLengthAt(clampedT);
        return stateAt(index, clampedT, s);
    }

    private PathState stateAt(int index, double t, double arcLength) {
        PathSegment segment = segments.get(index);
        Path path = segment.getPath();
        return new PathState(index, t, arcLength, path.getPoint(t), path.getUnitTangent(t),
                segment.headingAt(t), path.getCurvature(t));
    }

    /**
     * Projects {@code query} onto the chain and returns the closest state.
     *
     * <p>The search is restricted to the segment the robot is currently on and
     * the one after it. Searching the whole chain would let a robot that passes
     * near an earlier segment jump backwards -- which on a chain that loops back
     * past its own start means the follower decides it has un-driven half the
     * auto. Limiting the window makes progress monotonic in practice.
     *
     * @param currentSegment the segment the follower believes it is on
     * @param tGuess         the previous loop's parameter within that segment
     */
    public PathState project(Vector2d query, int currentSegment, double tGuess) {
        int from = Math.max(0, Math.min(currentSegment, segments.size() - 1));
        int to = Math.min(from + 1, segments.size() - 1);

        PathState best = null;
        double bestDistance = Double.MAX_VALUE;
        for (int i = from; i <= to; i++) {
            double guess = (i == from) ? tGuess : 0.0;
            double t = segments.get(i).getPath().getClosestT(query, guess);
            PathState state = stateAtSegment(i, t);
            double d = state.point.distanceTo(query);
            if (d < bestDistance) {
                bestDistance = d;
                best = state;
            }
        }
        return best;
    }

    /**
     * The speed cap in force at {@code arcLength} inches along the chain, or
     * {@code globalMax} where the segment there sets none.
     *
     * <p>This is what a constrained profile samples to build its velocity
     * ceiling.
     */
    public double maxVelocityAtArcLength(double arcLength, double globalMax) {
        double clamped = Math.max(0.0, Math.min(totalLength, arcLength));
        for (int i = 0; i < segments.size(); i++) {
            if (clamped <= startArcLength[i] + segments.get(i).length()) {
                double cap = segments.get(i).getMaxVelocity();
                return Double.isNaN(cap) ? globalMax : Math.min(cap, globalMax);
            }
        }
        double cap = segments.get(segments.size() - 1).getMaxVelocity();
        return Double.isNaN(cap) ? globalMax : Math.min(cap, globalMax);
    }

    /**
     * The signed curvature of the chain at {@code arcLength} inches, 1/inches.
     * Positive curves left.
     */
    public double curvatureAtArcLength(double arcLength) {
        return stateAtArcLength(arcLength).curvature;
    }

    /**
     * The full speed ceiling at {@code arcLength}: the global limit, this
     * segment's own cap if it has one, and whatever the corner's curvature
     * physically allows, whichever is lowest.
     *
     * <p>This is what a constrained profile should sample. The two-argument
     * {@link #maxVelocityAtArcLength} ignores curvature and is kept for callers
     * that only care about the configured caps.
     *
     * @param maxLateralAcceleration inches per second squared, or
     *                               {@link Double#POSITIVE_INFINITY} to ignore
     *                               curvature entirely
     */
    public double speedLimitAtArcLength(double arcLength, double globalMax,
                                        double maxLateralAcceleration) {
        double configured = maxVelocityAtArcLength(arcLength, globalMax);
        double cornering = com.platapus.pathing.control.ConstrainedProfile
                .lateralAccelerationLimit(curvatureAtArcLength(arcLength),
                        maxLateralAcceleration);
        return Math.min(configured, cornering);
    }

    /** Whether any segment sets its own speed cap. */
    public boolean hasVelocityOverrides() {
        for (PathSegment segment : segments) {
            if (segment.hasMaxVelocity()) {
                return true;
            }
        }
        return false;
    }

    /** The pose the chain starts from: its first point, with its first heading. */
    public PathState startState() {
        return stateAtSegment(0, 0.0);
    }

    /** The pose the chain ends at: its last point, with its final heading. */
    public PathState endState() {
        return stateAtSegment(segments.size() - 1, 1.0);
    }

    @Override
    public String toString() {
        return String.format("PathChain(%d segments, %.2f\")", segments.size(), totalLength);
    }
}
