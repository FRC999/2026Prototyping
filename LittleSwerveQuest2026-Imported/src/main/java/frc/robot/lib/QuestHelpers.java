// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.awt.geom.Point2D;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * This code is vibed via ChatGPT5
 * It helps determining Quest offset from the center of the robot's rotation
 */
public final class QuestHelpers {

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double clampRad(double v, double lo, double hi) { return clamp(v, lo, hi); }

    public static Matrix<N3, N1> questStdDev(double chassisSpeedMps) {
        double s = clamp(0.05 + 0.02 * chassisSpeedMps, 0.05, 0.15); // meters
        double yaw = clampRad(Units.degreesToRadians(1.0 + 0.5 * chassisSpeedMps),
                Units.degreesToRadians(0.7), Units.degreesToRadians(2.5));
        return VecBuilder.fill(s, s, yaw);
    }

    // ======================= CHARACTERIZATION ==========================

    /**
     * Estimate the center of the best-fit circle for points given as Double[2] =
     * {x, y}.
     * Requires at least 3 points. Robust to small noise; throws on
     * degenerate/collinear sets.
     *
     * Model: x^2 + y^2 + a x + b y + c = 0 -> center = (-a/2, -b/2)
     */
    public static Point2D.Double estimateCircleCenter(List<Double[]> points) {
        if (points == null || points.size() < 3) {
            throw new IllegalArgumentException("Need at least 3 (x,y) points.");
        }

        double sxx = 0, sxy = 0, sx = 0;
        double syy = 0, sy = 0, n = 0;
        double sx2y2 = 0, sx_x2y2 = 0, sy_x2y2 = 0;

        for (Double[] p : points) {
            if (p == null || p.length != 2)
                throw new IllegalArgumentException("Each element must be a Double[2] {x, y}.");
            double x = requireFinite(p[0], "x");
            double y = requireFinite(p[1], "y");
            double r2 = x * x + y * y;

            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            sx += x;
            sy += y;
            n += 1.0;

            // RHS sums for A^T b with b = -(x^2 + y^2)
            sx2y2 += -r2;
            sx_x2y2 += -r2 * x;
            sy_x2y2 += -r2 * y;
        }

        // Normal matrix M = A^T A, RHS v = A^T b, unknowns theta = [a, b, c]
        double[][] M = {
                { sxx, sxy, sx },
                { sxy, syy, sy },
                { sx, sy, n }
        };
        double[] v = { sx_x2y2, sy_x2y2, sx2y2 };

        double[] theta = solve3x3(M, v); // [a, b, c]
        return new Point2D.Double(-0.5 * theta[0], -0.5 * theta[1]);
    }

    private static double requireFinite(Double d, String name) {
        Objects.requireNonNull(d, name + " is null");
        if (!Double.isFinite(d)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
        return d;
    }

    /** Solve a 3x3 linear system with partial pivoting. Throws if singular. */
    private static double[] solve3x3(double[][] M, double[] v) {
        double[][] A = new double[3][4];
        for (int i = 0; i < 3; i++) {
            A[i][0] = M[i][0];
            A[i][1] = M[i][1];
            A[i][2] = M[i][2];
            A[i][3] = v[i];
        }
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            double maxAbs = Math.abs(A[col][col]);
            for (int r = col + 1; r < 3; r++) {
                double val = Math.abs(A[r][col]);
                if (val > maxAbs) {
                    maxAbs = val;
                    pivot = r;
                }
            }
            if (maxAbs < 1e-12) {
                throw new IllegalArgumentException("Degenerate/collinear points: cannot fit a unique circle.");
            }
            if (pivot != col) {
                double[] tmp = A[col];
                A[col] = A[pivot];
                A[pivot] = tmp;
            }
            double diag = A[col][col];
            for (int c = col; c < 4; c++)
                A[col][c] /= diag;
            for (int r = 0; r < 3; r++)
                if (r != col) {
                    double f = A[r][col];
                    if (f != 0)
                        for (int c = col; c < 4; c++)
                            A[r][c] -= f * A[col][c];
                }
        }
        return new double[] { A[0][3], A[1][3], A[2][3] };
    }
}
