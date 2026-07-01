package kinematics;

public class Kinematics {
    public static final boolean IK_AUDIT_ENABLED = false;

    // Các thông số của Robot 6DOF từ ARM.m
    public static final int NUM_JOINTS = 6;
    public static final double L0 = 120.0;
    public static final double L1 = 5.0;
    public static final double L2 = 10.0;
    public static final double L3 = 10.0;
    public static final double L4 = 20.0;
    public static final double L5 = 20.0;
    public static final double L6 = 10.0;
    public static final double L7 = 10.0;

    public static final String[] JOINT_NAMES = { "Khớp 1", "Khớp 2", "Khớp 3", "Khớp 4", "Khớp 5", "Khớp 6" };
    public static final double[] JOINT_MIN_RIGHT = { -45, -90, -90, -140, -90, -90 };
    public static final double[] JOINT_MAX_RIGHT = { 45, 90, 90, -30, 90, 90 };

    public static final double[] JOINT_MIN_LEFT = { -45, -90, -90, 30, -90, -90 };
    public static final double[] JOINT_MAX_LEFT = { 45, 90, 90, 140, 90, 90 };

    public static final double[] JOINT_MIN = JOINT_MIN_RIGHT;
    public static final double[] JOINT_MAX = JOINT_MAX_RIGHT;

    /**
     * Solve inverse kinematics using Damped Least Squares (DLS) numerical loop.
     */
    public static double[] solveIK(double px, double py, double pz, double[][] R_target, double[] qInitRad) {
        return solveIK(px, py, pz, R_target, qInitRad, true); // default to Right arm
    }

    public static double[] solveIK(double px, double py, double pz, double[][] R_target, double[] qInitRad, boolean isRight) {
        double[] q = qInitRad.clone();
        int maxIter = 100;
        double tolPos = 1.0; 
        double tolOri = 0.005; 
        double L_CHAR = (L2 + L3) + L4; 
        
        double[] p_d = { px, py, pz };
        double[] minLimRad = new double[NUM_JOINTS];
        double[] maxLimRad = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            minLimRad[i] = Math.toRadians(isRight ? JOINT_MIN_RIGHT[i] : JOINT_MIN_LEFT[i]);
            maxLimRad[i] = Math.toRadians(isRight ? JOINT_MAX_RIGHT[i] : JOINT_MAX_LEFT[i]);
        }

        double[] n_d = { R_target[0][0], R_target[1][0], R_target[2][0] };
        double[] s_d = { R_target[0][1], R_target[1][1], R_target[2][1] };
        double[] a_d = { R_target[0][2], R_target[1][2], R_target[2][2] };

        for (int iter = 0; iter < maxIter; iter++) {
            double[][] T_e = computeFKMatrix(q, isRight);
            double[] p_e = { T_e[0][3], T_e[1][3], T_e[2][3] };
            double[][] R_e = extractRotation(T_e);

            double[] e_p = { p_d[0] - p_e[0], p_d[1] - p_e[1], p_d[2] - p_e[2] };
            double errPosNorm = Math.sqrt(e_p[0]*e_p[0] + e_p[1]*e_p[1] + e_p[2]*e_p[2]);

            double[] n = { R_e[0][0], R_e[1][0], R_e[2][0] };
            double[] s = { R_e[0][1], R_e[1][1], R_e[2][1] };
            double[] a_vec = { R_e[0][2], R_e[1][2], R_e[2][2] };

            double[] crossN = crossProduct(n, n_d);
            double[] crossS = crossProduct(s, s_d);
            double[] crossA = crossProduct(a_vec, a_d);

            double[] e_o = {
                0.5 * (crossN[0] + crossS[0] + crossA[0]),
                0.5 * (crossN[1] + crossS[1] + crossA[1]),
                0.5 * (crossN[2] + crossS[2] + crossA[2])
            };
            double errOriNorm = Math.sqrt(e_o[0]*e_o[0] + e_o[1]*e_o[1] + e_o[2]*e_o[2]);

            if (errPosNorm < tolPos && errOriNorm < tolOri) {
                return convertToDegreesWrap(q);
            }

            double[][] J = computeJacobianEE(q, isRight);
            double detJ = compute6x6Determinant(J);
            double manipulability = Math.abs(detJ);

            double lambdaBase = 0.01;
            double lambdaMax = 0.4;
            double wThresh = 0.008;
            double lambda = lambdaBase;
            if (manipulability < wThresh) {
                double ratio = manipulability / wThresh;
                lambda = Math.sqrt(lambdaBase * lambdaBase + lambdaMax * lambdaMax * (1.0 - ratio) * (1.0 - ratio));
            }

            double[] e_unified = new double[6];
            System.arraycopy(e_p, 0, e_unified, 0, 3);
            e_unified[3] = L_CHAR * e_o[0];
            e_unified[4] = L_CHAR * e_o[1];
            e_unified[5] = L_CHAR * e_o[2];

            double[] dq = solveDLS(J, e_unified, lambda);

            double alphaStep = 1.0;
            double beta = 0.5;
            double currentError = errPosNorm + L_CHAR * errOriNorm;
            boolean stepAccepted = false;
            double[] q_cand = new double[NUM_JOINTS];

            for (int backstep = 0; backstep < 4; backstep++) {
                for (int i = 0; i < NUM_JOINTS; i++) {
                    q_cand[i] = Math.max(minLimRad[i], Math.min(maxLimRad[i], q[i] + alphaStep * dq[i]));
                }

                double[][] T_cand = computeFKMatrix(q_cand, isRight);
                double[] p_cand = { T_cand[0][3], T_cand[1][3], T_cand[2][3] };
                double errPosCand = Math.sqrt(
                    Math.pow(p_d[0]-p_cand[0], 2) + Math.pow(p_d[1]-p_cand[1], 2) + Math.pow(p_d[2]-p_cand[2], 2)
                );

                double[][] R_cand = extractRotation(T_cand);
                double[] n_c = { R_cand[0][0], R_cand[1][0], R_cand[2][0] };
                double[] s_c = { R_cand[0][1], R_cand[1][1], R_cand[2][1] };
                double[] a_c = { R_cand[0][2], R_cand[1][2], R_cand[2][2] };

                double[] cN = crossProduct(n_c, n_d);
                double[] cS = crossProduct(s_c, s_d);
                double[] cA = crossProduct(a_c, a_d);

                double[] e_o_cand = {
                    0.5 * (cN[0] + cS[0] + cA[0]),
                    0.5 * (cN[1] + cS[1] + cA[1]),
                    0.5 * (cN[2] + cS[2] + cA[2])
                };
                double errOriCand = Math.sqrt(e_o_cand[0]*e_o_cand[0] + e_o_cand[1]*e_o_cand[1] + e_o_cand[2]*e_o_cand[2]);

                double candidateError = errPosCand + L_CHAR * errOriCand;
                if (candidateError < currentError) {
                    q = q_cand.clone();
                    stepAccepted = true;
                    break;
                }
                alphaStep *= beta;
            }

            if (!stepAccepted) {
                for (int i = 0; i < NUM_JOINTS; i++) {
                    q[i] = Math.max(minLimRad[i], Math.min(maxLimRad[i], q[i] + 0.05 * dq[i]));
                }
            }
        }

        double[][] T_final = computeFKMatrix(q, isRight);
        double[] p_f = { T_final[0][3], T_final[1][3], T_final[2][3] };
        double finalPosErr = Math.sqrt(
            Math.pow(p_d[0]-p_f[0], 2) + Math.pow(p_d[1]-p_f[1], 2) + Math.pow(p_d[2]-p_f[2], 2)
        );

        double[][] R_f = extractRotation(T_final);
        double[] n_f = { R_f[0][0], R_f[1][0], R_f[2][0] };
        double[] s_f = { R_f[0][1], R_f[1][1], R_f[2][1] };
        double[] a_f = { R_f[0][2], R_f[1][2], R_f[2][2] };

        double[] cN_f = crossProduct(n_f, n_d);
        double[] cS_f = crossProduct(s_f, s_d);
        double[] cA_f = crossProduct(a_f, a_d);

        double[] final_e_o = {
            0.5 * (cN_f[0] + cS_f[0] + cA_f[0]),
            0.5 * (cN_f[1] + cS_f[1] + cA_f[1]),
            0.5 * (cN_f[2] + cS_f[2] + cA_f[2])
        };
        double finalOriErr = Math.sqrt(final_e_o[0]*final_e_o[0] + final_e_o[1]*final_e_o[1] + final_e_o[2]*final_e_o[2]);

        // Constraint Relaxation: accept if position is very close and orientation deviation is within ~5 degrees
        double maxAllowedOriDeviation = Math.toRadians(5.0);
        if (finalPosErr < 1.5 && finalOriErr < Math.sin(maxAllowedOriDeviation)) {
            return convertToDegreesWrap(q);
        }

        return null;
    }

    /**
     * Overloaded solveIK for compatibility with the existing calling convention.
     */
    public static double[] solveIK(double px, double py, double pz, double[][] R_target, String config,
            String configElbow) {
        double[] qInit = new double[NUM_JOINTS];
        double phi = Math.atan2(py, px);

        if (config.equals("+")) {
            qInit[0] = phi;
            qInit[1] = 0.5;
            qInit[2] = -0.5;
        } else {
            qInit[0] = phi + Math.PI;
            qInit[1] = -0.5;
            qInit[2] = 0.5;
        }
        if (configElbow.equals("-")) {
            qInit[2] = -qInit[2];
        }

        return solveIK(px, py, pz, R_target, qInit);
    }

    public static double[][] computeFKMatrix(double[] q) {
        return computeFKMatrix(q, true);
    }

    public static double[][] computeFKMatrix(double[] q, boolean isRight) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, q[0] },
                { -Math.PI / 2, d2, 0, -Math.PI / 2, q[1] },
                { -Math.PI / 2, 0, 0, -Math.PI, q[2] },
                { 0, 0, L4, -Math.PI / 2, q[3] },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, q[4] },
                { -Math.PI / 2, 0, 0, 0, q[5] }
        };
        for (int i = 0; i < NUM_JOINTS; i++) {
            T = multiply4x4(T, getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
        }
        T = multiply4x4(T, getToolMatrix());
        return T;
    }

    public static double[][] computeJacobianEE(double[] q) {
        return computeJacobianEE(q, true);
    }

    public static double[][] computeJacobianEE(double[] q, boolean isRight) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, q[0] },
                { -Math.PI / 2, d2, 0, -Math.PI / 2, q[1] },
                { -Math.PI / 2, 0, 0, -Math.PI, q[2] },
                { 0, 0, L4, -Math.PI / 2, q[3] },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, q[4] },
                { -Math.PI / 2, 0, 0, 0, q[5] }
        };

        double[][] z0 = new double[NUM_JOINTS][3];
        double[][] p0 = new double[NUM_JOINTS][3];

        for (int i = 0; i < NUM_JOINTS; i++) {
            double alpha = params[i][0];
            double a = params[i][2];
            double d = params[i][1];
            double offset = params[i][3];
            double qi = params[i][4];

            // intermediate frame T_i' = T * Rx(alpha) * Tx(a)
            double ca = Math.cos(alpha), sa = Math.sin(alpha);
            double[][] RxTx = {
                    { 1, 0, 0, a },
                    { 0, ca, -sa, 0 },
                    { 0, sa, ca, 0 },
                    { 0, 0, 0, 1 }
            };
            double[][] T_i_prime = multiply4x4(T, RxTx);

            // Joint i's axis in base frame
            z0[i][0] = T_i_prime[0][2];
            z0[i][1] = T_i_prime[1][2];
            z0[i][2] = T_i_prime[2][2];

            // Joint i's origin in base frame
            p0[i][0] = T_i_prime[0][3];
            p0[i][1] = T_i_prime[1][3];
            p0[i][2] = T_i_prime[2][3];

            // Complete transition T = T_i_prime * Rz(theta) * Tz(d)
            double theta = qi + offset;
            double ct = Math.cos(theta), st = Math.sin(theta);
            double[][] RzTz = {
                    { ct, -st, 0, 0 },
                    { st, ct, 0, 0 },
                    { 0, 0, 1, d },
                    { 0, 0, 0, 1 }
            };
            T = multiply4x4(T_i_prime, RzTz);
        }

        double[][] T_tool = multiply4x4(T, getToolMatrix());
        double[] p_tool = { T_tool[0][3], T_tool[1][3], T_tool[2][3] };
        double[][] R_tool = extractRotation(T_tool);

        // Construct Jacobian in base frame J0
        double[][] J0 = new double[6][NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            double dx = p_tool[0] - p0[i][0];
            double dy = p_tool[1] - p0[i][1];
            double dz = p_tool[2] - p0[i][2];

            // z0[i] x (p_tool - p0[i])
            J0[0][i] = z0[i][1] * dz - z0[i][2] * dy;
            J0[1][i] = z0[i][2] * dx - z0[i][0] * dz;
            J0[2][i] = z0[i][0] * dy - z0[i][1] * dx;

            J0[3][i] = z0[i][0];
            J0[4][i] = z0[i][1];
            J0[5][i] = z0[i][2];
        }

        // Rotate Jacobian to end-effector frame: Je = R_tool^T * J0
        double[][] RT = transpose3x3(R_tool);
        double[][] Je = new double[6][NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; j++) {
            Je[0][j] = RT[0][0] * J0[0][j] + RT[0][1] * J0[1][j] + RT[0][2] * J0[2][j];
            Je[1][j] = RT[1][0] * J0[0][j] + RT[1][1] * J0[1][j] + RT[1][2] * J0[2][j];
            Je[2][j] = RT[2][0] * J0[0][j] + RT[2][1] * J0[1][j] + RT[2][2] * J0[2][j];

            Je[3][j] = RT[0][0] * J0[3][j] + RT[0][1] * J0[4][j] + RT[0][2] * J0[5][j];
            Je[4][j] = RT[1][0] * J0[3][j] + RT[1][1] * J0[4][j] + RT[1][2] * J0[5][j];
            Je[5][j] = RT[2][0] * J0[3][j] + RT[2][1] * J0[4][j] + RT[2][2] * J0[5][j];
        }
        return Je;
    }

    public static double[] computeTr2Delta(double[][] T0, double[][] T1) {
        double[][] R0 = extractRotation(T0);
        double[][] R1 = extractRotation(T1);
        double[][] R0T = transpose3x3(R0);

        double[][] R = multiplyMatrices(R0T, R1);

        double dx = T1[0][3] - T0[0][3];
        double dy = T1[1][3] - T0[1][3];
        double dz = T1[2][3] - T0[2][3];
        double[] dp = {
                R0T[0][0] * dx + R0T[0][1] * dy + R0T[0][2] * dz,
                R0T[1][0] * dx + R0T[1][1] * dy + R0T[1][2] * dz,
                R0T[2][0] * dx + R0T[2][1] * dy + R0T[2][2] * dz
        };

        double[] dw = {
                0.5 * (R[2][1] - R[1][2]),
                0.5 * (R[0][2] - R[2][0]),
                0.5 * (R[1][0] - R[0][1])
        };

        return new double[] { dp[0], dp[1], dp[2], dw[0], dw[1], dw[2] };
    }

    public static double[] solveDLS(double[][] J, double[] e, double lambda) {
        int n = 6;
        double[][] JJT = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0;
                for (int k = 0; k < NUM_JOINTS; k++) {
                    sum += J[i][k] * J[j][k];
                }
                JJT[i][j] = sum;
            }
            JJT[i][i] += lambda * lambda;
        }

        double[] x = solveLinearSystem(JJT, e);
        if (x == null)
            return new double[NUM_JOINTS];

        double[] dq = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            double sum = 0;
            for (int k = 0; k < n; k++) {
                sum += J[k][i] * x[k];
            }
            dq[i] = sum;
        }
        return dq;
    }

    private static double[] solveLinearSystem(double[][] A, double[] b) {
        int n = b.length;
        double[][] M = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, M[i], 0, n);
            M[i][n] = b[i];
        }

        for (int i = 0; i < n; i++) {
            int max = i;
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(M[k][i]) > Math.abs(M[max][i])) {
                    max = k;
                }
            }
            double[] temp = M[i];
            M[i] = M[max];
            M[max] = temp;

            if (Math.abs(M[i][i]) < 1e-12) {
                return null;
            }

            for (int k = i + 1; k < n; k++) {
                double factor = M[k][i] / M[i][i];
                for (int j = i; j <= n; j++) {
                    M[k][j] -= factor * M[i][j];
                }
            }
        }

        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0;
            for (int j = i + 1; j < n; j++) {
                sum += M[i][j] * x[j];
            }
            x[i] = (M[i][n] - sum) / M[i][i];
        }
        return x;
    }

    public static double wrapToPi(double rad) {
        while (rad > Math.PI)
            rad -= 2 * Math.PI;
        while (rad < -Math.PI)
            rad += 2 * Math.PI;
        return rad;
    }

    public static double[][] getMDHMatrix(double alpha, double d, double a, double offset, double q) {
        double theta = q + offset;
        double ct = Math.cos(theta), st = Math.sin(theta);
        double ca = Math.cos(alpha), sa = Math.sin(alpha);

        return new double[][] {
                { ct, -st, 0, a },
                { st * ca, ct * ca, -sa, -sa * d },
                { st * sa, ct * sa, ca, ca * d },
                { 0, 0, 0, 1 }
        };
    }

    public static double[][] getToolMatrix() {
        return new double[][] {
                { 1, 0, 0, 0 },
                { 0, 0, -1, -L7 },
                { 0, 1, 0, 0 },
                { 0, 0, 0, 1 }
        };
    }

    public static double[][] invertMatrix4x4(double[][] M) {
        double[][] R = extractRotation(M);
        double[][] RT = transpose3x3(R);
        double[] p = { M[0][3], M[1][3], M[2][3] };
        double[] negRTp = {
                -(RT[0][0] * p[0] + RT[0][1] * p[1] + RT[0][2] * p[2]),
                -(RT[1][0] * p[0] + RT[1][1] * p[1] + RT[1][2] * p[2]),
                -(RT[2][0] * p[0] + RT[2][1] * p[1] + RT[2][2] * p[2])
        };
        return new double[][] {
                { RT[0][0], RT[0][1], RT[0][2], negRTp[0] },
                { RT[1][0], RT[1][1], RT[1][2], negRTp[1] },
                { RT[2][0], RT[2][1], RT[2][2], negRTp[2] },
                { 0, 0, 0, 1 }
        };
    }

    public static double[][] multiply4x4(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                if (A[i][k] != 0)
                    for (int j = 0; j < 4; j++)
                        C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    public static double[][] multiplyMatrices(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int k = 0; k < 3; k++)
                if (A[i][k] != 0)
                    for (int j = 0; j < 3; j++)
                        C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    public static double[][] transpose3x3(double[][] A) {
        return new double[][] { { A[0][0], A[1][0], A[2][0] }, { A[0][1], A[1][1], A[2][1] },
                { A[0][2], A[1][2], A[2][2] } };
    }

    public static double[][] extractRotation(double[][] T) {
        return new double[][] { { T[0][0], T[0][1], T[0][2] }, { T[1][0], T[1][1], T[1][2] },
                { T[2][0], T[2][1], T[2][2] } };
    }

    private static double[] convertToDegreesWrap(double[] q) {
        double[] q_deg = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            double deg = Math.toDegrees(q[i]);
            while (deg > 180) deg -= 360;
            while (deg < -180) deg += 360;
            q_deg[i] = deg;
        }
        return q_deg;
    }

    public static double[] crossProduct(double[] u, double[] v) {
        return new double[] {
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0]
        };
    }

    public static double compute6x6Determinant(double[][] A) {
        int n = 6;
        double[][] M = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, M[i], 0, n);
        }

        double det = 1.0;
        int swaps = 0;
        for (int i = 0; i < n; i++) {
            int pivotRow = i;
            for (int r = i + 1; r < n; r++) {
                if (Math.abs(M[r][i]) > Math.abs(M[pivotRow][i])) {
                    pivotRow = r;
                }
            }
            if (pivotRow != i) {
                double[] temp = M[i];
                M[i] = M[pivotRow];
                M[pivotRow] = temp;
                swaps++;
            }
            if (Math.abs(M[i][i]) < 1e-12) {
                return 0.0;
            }
            det *= M[i][i];
            for (int r = i + 1; r < n; r++) {
                double factor = M[r][i] / M[i][i];
                for (int c = i; c < n; c++) {
                    M[r][c] -= factor * M[i][c];
                }
            }
        }
        if (swaps % 2 == 1) {
            det = -det;
        }
        return det;
    }
}
