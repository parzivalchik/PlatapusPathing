package com.platapus.pathing.math;

/**
 * A 3x3 matrix, just enough of one for the library's three-parameter estimators.
 *
 * <p>Deliberately not a general linear algebra library. The filter's state is
 * {@code [x, y, heading]} and will stay that size, so a fixed 3x3 with hand-rolled
 * operations is faster than anything generic, allocates predictably, and adds no
 * dependency to a module that has to stay dependency-free.
 *
 * <p>Immutable: every operation returns a new matrix.
 */
public final class Matrix3 {

    private final double[] m; // row-major, 9 entries

    private Matrix3(double[] m) {
        this.m = m;
    }

    public static Matrix3 of(double a, double b, double c,
                      double d, double e, double f,
                      double g, double h, double i) {
        return new Matrix3(new double[]{a, b, c, d, e, f, g, h, i});
    }

    public static Matrix3 identity() {
        return of(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    public static Matrix3 zero() {
        return new Matrix3(new double[9]);
    }

    public static Matrix3 diagonal(double a, double b, double c) {
        return of(a, 0, 0, 0, b, 0, 0, 0, c);
    }

    public double get(int row, int col) {
        return m[row * 3 + col];
    }

    public Matrix3 plus(Matrix3 o) {
        double[] out = new double[9];
        for (int i = 0; i < 9; i++) {
            out[i] = m[i] + o.m[i];
        }
        return new Matrix3(out);
    }

    public Matrix3 minus(Matrix3 o) {
        double[] out = new double[9];
        for (int i = 0; i < 9; i++) {
            out[i] = m[i] - o.m[i];
        }
        return new Matrix3(out);
    }

    public Matrix3 times(Matrix3 o) {
        double[] out = new double[9];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                double sum = 0;
                for (int k = 0; k < 3; k++) {
                    sum += m[r * 3 + k] * o.m[k * 3 + c];
                }
                out[r * 3 + c] = sum;
            }
        }
        return new Matrix3(out);
    }

    public Matrix3 times(double s) {
        double[] out = new double[9];
        for (int i = 0; i < 9; i++) {
            out[i] = m[i] * s;
        }
        return new Matrix3(out);
    }

    /** Matrix-vector product. {@code v} must have length 3. */
    public double[] times(double[] v) {
        return new double[]{
                m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
                m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
                m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
        };
    }

    public Matrix3 transpose() {
        return of(m[0], m[3], m[6],
                  m[1], m[4], m[7],
                  m[2], m[5], m[8]);
    }

    public double determinant() {
        return m[0] * (m[4] * m[8] - m[5] * m[7])
                - m[1] * (m[3] * m[8] - m[5] * m[6])
                + m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    /**
     * The inverse, by cofactors.
     *
     * @throws IllegalStateException if the matrix is singular. In this filter the
     *         only matrix ever inverted is an innovation covariance, which is the
     *         sum of two positive-definite matrices and so cannot be singular
     *         unless a caller passed a zero or negative measurement variance --
     *         which is a bug worth hearing about rather than absorbing.
     */
    public Matrix3 inverse() {
        double det = determinant();
        if (Math.abs(det) < 1e-15) {
            throw new IllegalStateException(
                    "singular matrix; check that measurement variances are positive");
        }
        double invDet = 1.0 / det;
        return of(
                (m[4] * m[8] - m[5] * m[7]) * invDet,
                (m[2] * m[7] - m[1] * m[8]) * invDet,
                (m[1] * m[5] - m[2] * m[4]) * invDet,
                (m[5] * m[6] - m[3] * m[8]) * invDet,
                (m[0] * m[8] - m[2] * m[6]) * invDet,
                (m[2] * m[3] - m[0] * m[5]) * invDet,
                (m[3] * m[7] - m[4] * m[6]) * invDet,
                (m[1] * m[6] - m[0] * m[7]) * invDet,
                (m[0] * m[4] - m[1] * m[3]) * invDet);
    }

    /**
     * Averages this matrix with its own transpose.
     *
     * <p>A covariance is symmetric by definition, but repeated floating-point
     * updates let the two halves drift apart by a few ulps. Left alone that
     * asymmetry compounds and can eventually push the covariance
     * non-positive-definite, at which point the filter starts producing
     * nonsense. Re-symmetrising after each update is the standard cheap guard.
     */
    public Matrix3 symmetrized() {
        return plus(transpose()).times(0.5);
    }

    /**
     * The outer product {@code a * b^T}. Used by the recursive least squares
     * update, where the correction to the covariance is exactly this shape.
     */
    public static Matrix3 outer(double[] a, double[] b) {
        return of(a[0] * b[0], a[0] * b[1], a[0] * b[2],
                  a[1] * b[0], a[1] * b[1], a[1] * b[2],
                  a[2] * b[0], a[2] * b[1], a[2] * b[2]);
    }

    /** The dot product of two length-3 vectors. */
    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    @Override
    public String toString() {
        return String.format("[%.4f %.4f %.4f; %.4f %.4f %.4f; %.4f %.4f %.4f]",
                m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    }
}
