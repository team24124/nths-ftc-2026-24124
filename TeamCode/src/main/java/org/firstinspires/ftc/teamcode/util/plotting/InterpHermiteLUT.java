package org.firstinspires.ftc.teamcode.util.plotting;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Cubic Hermite Spline:
 * - Accepts unsorted input points (build() sorts)
 * - Validates inputs
 * - Avoids brittle double-equality checks
 * - Requires build() after adding points; keeps a built flag
 * - Optional array constructors for convenience
 */
public class InterpHermiteLUT {

    public enum Extrapolation { CLAMP, LINEAR }

    // High precision tolerance suitable for FTC use
    private static final double EPS = 1e-12;

    private final List<Double> xs = new ArrayList<>();
    private final List<Double> ys = new ArrayList<>();
    private final List<Double> slopes = new ArrayList<>();
    private Extrapolation extrapolation = Extrapolation.LINEAR;
    private boolean isBuilt = false;

    public InterpHermiteLUT() {}

    public InterpHermiteLUT(double[] xs, double[] ys) {
        if (xs.length != ys.length) {
            throw new IllegalArgumentException("xs and ys arrays must have same length");
        }
        for (int i = 0; i < xs.length; i++) {
            addPoint(xs[i], ys[i]);
        }
    }

    public InterpHermiteLUT(double[][] points) {
        for (double[] pt : points) {
            if (pt.length != 2) {
                throw new IllegalArgumentException("Each point must be [x, y]");
            }
            addPoint(pt[0], pt[1]);
        }
    }

    public void setExtrapolation(Extrapolation method) {
        this.extrapolation = method;
    }

    public void addPoint(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("x and y must be finite numbers");
        }
        xs.add(x);
        ys.add(y);
        isBuilt = false;
        slopes.clear();
    }

    public void clear() {
        xs.clear();
        ys.clear();
        slopes.clear();
        isBuilt = false;
    }

    public int size() {
        return xs.size();
    }

    public double getX(int i) {
        return xs.get(i);
    }

    public double getY(int i) {
        return ys.get(i);
    }

    public double getSlope(int i) {
        if (!isBuilt) throw new IllegalStateException("Spline not built - call build()");
        return slopes.get(i);
    }

    public void build() {
        int n = xs.size();
        if (n < 2) throw new IllegalStateException("Need at least two points to build spline");

        List<Point> pts = new ArrayList<>(n);
        for (int i = 0; i < n; i++) pts.add(new Point(xs.get(i), ys.get(i)));
        pts.sort(Comparator.comparingDouble(p -> p.x));

        xs.clear();
        ys.clear();
        for (Point p : pts) {
            if (!xs.isEmpty()) {
                double lastX = xs.get(xs.size() - 1);
                if (Math.abs(p.x - lastX) <= EPS) {
                    throw new IllegalStateException(
                            String.format("Duplicate x detected (within eps=%.1e): %s and %s", EPS, lastX, p.x));
                }
            }
            xs.add(p.x);
            ys.add(p.y);
        }

        n = xs.size();
        slopes.clear();

        for (int i = 0; i < n; i++) {
            double m;
            if (i == 0) {
                double dx = xs.get(1) - xs.get(0);
                m = Math.abs(dx) <= EPS ? 0.0 : (ys.get(1) - ys.get(0)) / dx;
            } else if (i == n - 1) {
                double dx = xs.get(n - 1) - xs.get(n - 2);
                m = Math.abs(dx) <= EPS ? 0.0 : (ys.get(n - 1) - ys.get(n - 2)) / dx;
            } else {
                double dx1 = xs.get(i) - xs.get(i - 1);
                double dx2 = xs.get(i + 1) - xs.get(i);
                double dy1 = ys.get(i) - ys.get(i - 1);
                double dy2 = ys.get(i + 1) - ys.get(i);
                if (Math.abs(dx1) <= EPS || Math.abs(dx2) <= EPS) {
                    m = 0.0;
                } else {
                    m = (dx1 * (dy2 / dx2) + dx2 * (dy1 / dx1)) / (dx1 + dx2);
                }
            }
            slopes.add(m);
        }

        isBuilt = true;
    }

    public double get(double x) {
        if (!isBuilt) throw new IllegalStateException("Spline not built - call build()");
        int n = xs.size();
        if (n < 2) throw new IllegalStateException("Spline has insufficient points");

        int idx = Collections.binarySearch(xs, x);
        if (idx >= 0) return ys.get(idx);

        int insert = -idx - 1;
        if (insert == 0) {
            if (extrapolation == Extrapolation.CLAMP) return ys.get(0);
            return ys.get(0) + slopes.get(0) * (x - xs.get(0));
        }
        if (insert >= n) {
            if (extrapolation == Extrapolation.CLAMP) return ys.get(n - 1);
            return ys.get(n - 1) + slopes.get(n - 1) * (x - xs.get(n - 1));
        }

        int i0 = insert - 1;
        int i1 = insert;
        return hermite(x, xs.get(i0), ys.get(i0), slopes.get(i0),
                xs.get(i1), ys.get(i1), slopes.get(i1));
    }

    public double[][] plot(int numPoints) {
        if (!isBuilt) throw new IllegalStateException("Spline not built - call build()");
        if (numPoints < 2) throw new IllegalArgumentException("numPoints must be at least 2");

        double xMin = xs.get(0);
        double xMax = xs.get(xs.size() - 1);
        double[][] result = new double[numPoints][2];

        for (int i = 0; i < numPoints; i++) {
            double t = i / (numPoints - 1.0);
            double x = xMin + t * (xMax - xMin);
            result[i][0] = x;
            result[i][1] = get(x);
        }

        return result;
    }

    public double[][] plot(double xMin, double xMax, int numPoints) {
        if (!isBuilt) throw new IllegalStateException("Spline not built - call build()");
        if (numPoints < 2) throw new IllegalArgumentException("numPoints must be at least 2");
        if (xMax <= xMin) throw new IllegalArgumentException("xMax must be greater than xMin");

        double[][] result = new double[numPoints][2];
        for (int i = 0; i < numPoints; i++) {
            double t = i / (numPoints - 1.0);
            double x = xMin + t * (xMax - xMin);
            result[i][0] = x;
            result[i][1] = get(x);
        }
        return result;
    }

    private double hermite(double x, double x0, double y0, double m0,
                           double x1, double y1, double m1) {
        double dx = x1 - x0;
        if (Math.abs(dx) < EPS) return y0;  // Safety fallback

        double t = (x - x0) / dx;
        double t2 = t * t;
        double t3 = t2 * t;
        double h00 = 2*t3 - 3*t2 + 1;
        double h10 = t3 - 2*t2 + t;
        double h01 = -2*t3 + 3*t2;
        double h11 = t3 - t2;
        return h00 * y0 + h10 * dx * m0 + h01 * y1 + h11 * dx * m1;
    }

    private static class Point {
        final double x, y;
        Point(double x, double y) { this.x = x; this.y = y; }
    }
}