package kinematics;

public class Kinematics {
    public static final boolean IK_AUDIT_ENABLED = false;
    public static int solverMode = JniKinematics.isLoaded() ? 1 : 0; // 0 = Java Numerical, 1 = C++ JNI Numerical, 2 =
                                                                     // C++ JNI IKFast

    // Các thông số của Robot 6DOF từ ARM.m
    public static final int NUM_JOINTS = 6;
    public static final double L0 = 130.0;
    public static final double L1 = 0.0;
    public static final double L2 = 32.0;
    public static final double L3 = 0.0;
    public static final double L4 = 20.0;
    public static final double L5 = 25.0;
    public static final double L6 = 0.0;
    public static final double L7 = 15.0;

    public static final String[] JOINT_NAMES = { "Khớp 1", "Khớp 2", "Khớp 3", "Khớp 4", "Khớp 5", "Khớp 6" };
    public static final double[] JOINT_MIN_RIGHT = { -45, -90, 20, -95, -90, 0 };
    public static final double[] JOINT_MAX_RIGHT = { 45, 90, 165, -15, 90, 90 };

    public static final double[] JOINT_MIN_LEFT = { -45, -90, -165, 15, -90, 0 };
    public static final double[] JOINT_MAX_LEFT = { 45, 90, -20, 95, 90, 90 };

    public static final double[] JOINT_MIN = JOINT_MIN_RIGHT;
    public static final double[] JOINT_MAX = JOINT_MAX_RIGHT;

    /**
     * Solve inverse kinematics using Damped Least Squares (DLS) numerical loop.
     */
    public static double[] solveIK(double px, double py, double pz, double[][] R_target, double[] qInitRad) {
        return solveIK(px, py, pz, R_target, qInitRad, true); // default to Right arm
    }

    public static double[] solveIK(double px, double py, double pz, double[][] R_target, double[] qInitRad,
            boolean isRight) {
        if (solverMode > 0 && JniKinematics.isLoaded()) {
            double[] sol = JniKinematics.solveIKNative(px, py, pz, R_target, qInitRad, isRight, solverMode);
            if (sol != null) {
                if (isWithinLimits(sol, isRight)) {
                    return sol;
                }
            }
        }
        IKWorkspace ws = new IKWorkspace();
        double[] q = qInitRad.clone();
        double[] bestQ = q.clone();
        double bestErrNorm = Double.MAX_VALUE;

        double[][] T_target = {
                { R_target[0][0], R_target[0][1], R_target[0][2], px },
                { R_target[1][0], R_target[1][1], R_target[1][2], py },
                { R_target[2][0], R_target[2][1], R_target[2][2], pz },
                { 0, 0, 0, 1 }
        };

        int maxIter = 200;
        double tol = 1e-5;
        double alpha = 0.8; // Khởi đầu với tốc độ học cao hơn để bắt kịp quỹ đạo
        double prevErrNorm = Double.MAX_VALUE;

        double[] minLimRad = new double[NUM_JOINTS];
        double[] maxLimRad = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            minLimRad[i] = Math.toRadians(isRight ? JOINT_MIN_RIGHT[i] : JOINT_MIN_LEFT[i]);
            maxLimRad[i] = Math.toRadians(isRight ? JOINT_MAX_RIGHT[i] : JOINT_MAX_LEFT[i]);
        }

        for (int iter = 0; iter < maxIter; iter++) {
            computeFKMatrix(q, isRight, ws);
            computeTr2Delta(ws.T, T_target, ws);

            double errNorm = 0;
            for (int i = 0; i < 6; i++) {
                errNorm += ws.delta[i] * ws.delta[i];
            }
            errNorm = Math.sqrt(errNorm);

            if (errNorm < bestErrNorm) {
                bestErrNorm = errNorm;
                System.arraycopy(q, 0, bestQ, 0, NUM_JOINTS);
            }

            // --- CẢI TIẾN 1: ADAPTIVE STEP SIZE (ALPHA ĐỘNG) ---
            if (errNorm > prevErrNorm) {
                alpha *= 0.5;
            } else {
                alpha = Math.min(0.95, alpha * 1.05);
            }
            prevErrNorm = errNorm;

            if (errNorm < tol) {
                return convertToDegreesWrap(q);
            }

            computeJacobianEE(q, isRight, ws);

            // Adaptive DLS
            double detJ = compute6x6Determinant(ws.Je);
            double manipulability = Math.abs(detJ);
            double lambda = 0.01;
            if (manipulability < 0.008) {
                double ratio = manipulability / 0.008;
                lambda = Math.sqrt(0.01 * 0.01 + 0.4 * 0.4 * (1 - ratio) * (1 - ratio));
            }

            solveDLS(ws.Je, ws.delta, lambda, ws);

            // --- CẢI TIẾN 2: THUẬT TOÁN KẸP BIÊN GIẢM CHẤN (CLAMPING) ---
            for (int i = 0; i < NUM_JOINTS; i++) {
                double nextQ = wrapToPi(q[i] + alpha * ws.dq[i]);
                if (nextQ < minLimRad[i] || nextQ > maxLimRad[i]) {
                    ws.dq[i] = 0; // Khóa khớp này lại, ép các khớp khác gánh quỹ đạo
                    nextQ = Math.max(minLimRad[i], Math.min(maxLimRad[i], nextQ));
                }
                q[i] = nextQ;
            }
        }

        // Fallback nếu vượt quá số vòng lặp nhưng vị trí đã rất gần đích
        computeFKMatrix(bestQ, isRight, ws);
        double dx = ws.T[0][3] - px;
        double dy = ws.T[1][3] - py;
        double dz = ws.T[2][3] - pz;
        double posErr = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (posErr <= 0.30) {
            double[] res = convertToDegreesWrap(bestQ);
            if (isWithinLimits(res, isRight)) {
                return res;
            }
        }

        return null;
    }

    /**
     * Check if θ-space angles are within limits.
     */
    public static boolean isWithinLimits(double[] thetaDeg, boolean isRight) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            double min = isRight ? JOINT_MIN_RIGHT[i] : JOINT_MIN_LEFT[i];
            double max = isRight ? JOINT_MAX_RIGHT[i] : JOINT_MAX_LEFT[i];
            if (thetaDeg[i] < min - 0.1 || thetaDeg[i] > max + 0.1) {
                return false;
            }
        }
        return true;
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
                { -Math.PI / 2, L5 + L6, 0, 0, q[4] },
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
                { -Math.PI / 2, L5 + L6, 0, 0, q[4] },
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

        // Relative rotation in local (End-Effector) frame
        double[][] R = multiplyMatrices(R0T, R1);
        R = orthonormalize3x3(R);

        double dx = T1[0][3] - T0[0][3];
        double dy = T1[1][3] - T0[1][3];
        double dz = T1[2][3] - T0[2][3];

        // Translate world displacement to local frame
        double[] dp = {
                R0T[0][0] * dx + R0T[0][1] * dy + R0T[0][2] * dz,
                R0T[1][0] * dx + R0T[1][1] * dy + R0T[1][2] * dz,
                R0T[2][0] * dx + R0T[2][1] * dy + R0T[2][2] * dz
        };

        // Tính toán góc xoay theta từ Trace của ma trận R
        double trace = R[0][0] + R[1][1] + R[2][2];
        double cosTheta = 0.5 * (trace - 1.0);
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta)); // Giới hạn chống tràn số thực
        double theta = Math.acos(cosTheta);

        double[] dw = new double[3];
        if (theta < 1e-6) {
            dw[0] = 0;
            dw[1] = 0;
            dw[2] = 0;
        } else if (theta > 3.0) {
            // Tìm trục xoay dựa trên đường chéo chính của ma trận R khi theta -> pi
            dw[0] = theta * Math.sqrt(Math.max(0.0, 0.5 * (R[0][0] + 1.0)));
            dw[1] = theta * Math.sqrt(Math.max(0.0, 0.5 * (R[1][1] + 1.0)));
            dw[2] = theta * Math.sqrt(Math.max(0.0, 0.5 * (R[2][2] + 1.0)));

            // Khôi phục dấu từ các phần tử chéo phụ
            if (R[2][1] - R[1][2] < 0)
                dw[0] = -dw[0];
            if (R[0][2] - R[2][0] < 0)
                dw[1] = -dw[1];
            if (R[1][0] - R[0][1] < 0)
                dw[2] = -dw[2];
        } else {
            double sinTheta = Math.sin(theta);
            double s = (Math.abs(sinTheta) > 1e-4) ? (0.5 * theta / sinTheta) : 0.5;
            dw[0] = (R[2][1] - R[1][2]) * s;
            dw[1] = (R[0][2] - R[2][0]) * s;
            dw[2] = (R[1][0] - R[0][1]) * s;
        }

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
                { 0, -1, 0, 0 },
                { 0, 0, -1, -L7 },
                { 1, 0, 0, 0 },
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

    public static double[][] orthonormalize3x3(double[][] R) {
        double[][] Q = new double[3][3];
        double len0 = Math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0] + R[2][0] * R[2][0]);
        if (len0 < 1e-9)
            len0 = 1.0;
        Q[0][0] = R[0][0] / len0;
        Q[1][0] = R[1][0] / len0;
        Q[2][0] = R[2][0] / len0;

        double dot = Q[0][0] * R[0][1] + Q[1][0] * R[1][1] + Q[2][0] * R[2][1];
        double v1_x = R[0][1] - dot * Q[0][0];
        double v1_y = R[1][1] - dot * Q[1][0];
        double v1_z = R[2][1] - dot * Q[2][0];
        double len1 = Math.sqrt(v1_x * v1_x + v1_y * v1_y + v1_z * v1_z);
        if (len1 < 1e-9)
            len1 = 1.0;
        Q[0][1] = v1_x / len1;
        Q[1][1] = v1_y / len1;
        Q[2][1] = v1_z / len1;

        Q[0][2] = Q[1][0] * Q[2][1] - Q[2][0] * Q[1][1];
        Q[1][2] = Q[2][0] * Q[0][1] - Q[0][0] * Q[2][1];
        Q[2][2] = Q[0][0] * Q[1][1] - Q[1][0] * Q[0][1];

        return Q;
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
            while (deg > 180)
                deg -= 360;
            while (deg < -180)
                deg += 360;
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

    // --- ZERO-ALLOCATION IN-PLACE ALGORITHMS ---

    public static void computeFKMatrix(double[] q, boolean isRight, IKWorkspace ws) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                ws.T[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = ws.params;
        params[0][0] = 0;
        params[0][1] = L1 + L0;
        params[0][2] = 0;
        params[0][3] = -Math.PI / 2;
        params[0][4] = q[0];
        params[1][0] = -Math.PI / 2;
        params[1][1] = d2;
        params[1][2] = 0;
        params[1][3] = -Math.PI / 2;
        params[1][4] = q[1];
        params[2][0] = -Math.PI / 2;
        params[2][1] = 0;
        params[2][2] = 0;
        params[2][3] = -Math.PI;
        params[2][4] = q[2];
        params[3][0] = 0;
        params[3][1] = 0;
        params[3][2] = L4;
        params[3][3] = -Math.PI / 2;
        params[3][4] = q[3];
        params[4][0] = -Math.PI / 2;
        params[4][1] = L5 + L6;
        params[4][2] = 0;
        params[4][3] = 0;
        params[4][4] = q[4];
        params[5][0] = -Math.PI / 2;
        params[5][1] = 0;
        params[5][2] = 0;
        params[5][3] = 0;
        params[5][4] = q[5];

        for (int i = 0; i < NUM_JOINTS; i++) {
            getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4], ws.MDH);
            multiply4x4(ws.T, ws.MDH, ws.T_next);
            for (int r = 0; r < 4; r++) {
                System.arraycopy(ws.T_next[r], 0, ws.T[r], 0, 4);
            }
        }

        getToolMatrix(ws.MDH);
        multiply4x4(ws.T, ws.MDH, ws.T_next);
        for (int r = 0; r < 4; r++) {
            System.arraycopy(ws.T_next[r], 0, ws.T[r], 0, 4);
        }
    }

    public static void getMDHMatrix(double alpha, double d, double a, double offset, double q, double[][] out) {
        double theta = q + offset;
        double ct = Math.cos(theta), st = Math.sin(theta);
        double ca = Math.cos(alpha), sa = Math.sin(alpha);

        out[0][0] = ct;
        out[0][1] = -st;
        out[0][2] = 0;
        out[0][3] = a;
        out[1][0] = st * ca;
        out[1][1] = ct * ca;
        out[1][2] = -sa;
        out[1][3] = -sa * d;
        out[2][0] = st * sa;
        out[2][1] = ct * sa;
        out[2][2] = ca;
        out[2][3] = ca * d;
        out[3][0] = 0;
        out[3][1] = 0;
        out[3][2] = 0;
        out[3][3] = 1;
    }

    public static void getToolMatrix(double[][] out) {
        out[0][0] = 0;
        out[0][1] = -1;
        out[0][2] = 0;
        out[0][3] = 0;
        out[1][0] = 0;
        out[1][1] = 0;
        out[1][2] = -1;
        out[1][3] = -L7;
        out[2][0] = 1;
        out[2][1] = 0;
        out[2][2] = 0;
        out[2][3] = 0;
        out[3][0] = 0;
        out[3][1] = 0;
        out[3][2] = 0;
        out[3][3] = 1;
    }

    public static void multiply4x4(double[][] A, double[][] B, double[][] C) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                C[i][j] = 0;
            }
        }
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 4; k++) {
                double aik = A[i][k];
                if (aik != 0) {
                    for (int j = 0; j < 4; j++) {
                        C[i][j] += aik * B[k][j];
                    }
                }
            }
        }
    }

    public static void computeTr2Delta(double[][] T0, double[][] T1, IKWorkspace ws) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ws.R0[i][j] = T0[i][j];
                ws.R1[i][j] = T1[i][j];
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ws.R0T[i][j] = ws.R0[j][i];
            }
        }
        multiply3x3(ws.R0T, ws.R1, ws.R);
        orthonormalize3x3InPlace(ws.R);

        double dx = T1[0][3] - T0[0][3];
        double dy = T1[1][3] - T0[1][3];
        double dz = T1[2][3] - T0[2][3];

        ws.dp[0] = ws.R0T[0][0] * dx + ws.R0T[0][1] * dy + ws.R0T[0][2] * dz;
        ws.dp[1] = ws.R0T[1][0] * dx + ws.R0T[1][1] * dy + ws.R0T[1][2] * dz;
        ws.dp[2] = ws.R0T[2][0] * dx + ws.R0T[2][1] * dy + ws.R0T[2][2] * dz;

        double trace = ws.R[0][0] + ws.R[1][1] + ws.R[2][2];
        double cosTheta = 0.5 * (trace - 1.0);
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
        double theta = Math.acos(cosTheta);

        if (theta < 1e-6) {
            ws.dw[0] = 0;
            ws.dw[1] = 0;
            ws.dw[2] = 0;
        } else if (theta > 3.0) {
            ws.dw[0] = theta * Math.sqrt(Math.max(0.0, 0.5 * (ws.R[0][0] + 1.0)));
            ws.dw[1] = theta * Math.sqrt(Math.max(0.0, 0.5 * (ws.R[1][1] + 1.0)));
            ws.dw[2] = theta * Math.sqrt(Math.max(0.0, 0.5 * (ws.R[2][2] + 1.0)));

            if (ws.R[2][1] - ws.R[1][2] < 0)
                ws.dw[0] = -ws.dw[0];
            if (ws.R[0][2] - ws.R[2][0] < 0)
                ws.dw[1] = -ws.dw[1];
            if (ws.R[1][0] - ws.R[0][1] < 0)
                ws.dw[2] = -ws.dw[2];
        } else {
            double sinTheta = Math.sin(theta);
            double s = (Math.abs(sinTheta) > 1e-4) ? (0.5 * theta / sinTheta) : 0.5;
            ws.dw[0] = (ws.R[2][1] - ws.R[1][2]) * s;
            ws.dw[1] = (ws.R[0][2] - ws.R[2][0]) * s;
            ws.dw[2] = (ws.R[1][0] - ws.R[0][1]) * s;
        }

        ws.delta[0] = ws.dp[0];
        ws.delta[1] = ws.dp[1];
        ws.delta[2] = ws.dp[2];
        ws.delta[3] = ws.dw[0];
        ws.delta[4] = ws.dw[1];
        ws.delta[5] = ws.dw[2];
    }

    public static void multiply3x3(double[][] A, double[][] B, double[][] C) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
    }

    public static void orthonormalize3x3InPlace(double[][] R) {
        double len0 = Math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0] + R[2][0] * R[2][0]);
        if (len0 < 1e-9)
            len0 = 1.0;
        R[0][0] /= len0;
        R[1][0] /= len0;
        R[2][0] /= len0;

        double dot = R[0][0] * R[0][1] + R[1][0] * R[1][1] + R[2][0] * R[2][1];
        R[0][1] -= dot * R[0][0];
        R[1][1] -= dot * R[1][0];
        R[2][1] -= dot * R[2][0];
        double len1 = Math.sqrt(R[0][1] * R[0][1] + R[1][1] * R[1][1] + R[2][1] * R[2][1]);
        if (len1 < 1e-9)
            len1 = 1.0;
        R[0][1] /= len1;
        R[1][1] /= len1;
        R[2][1] /= len1;

        R[0][2] = R[1][0] * R[2][1] - R[2][0] * R[1][1];
        R[1][2] = R[2][0] * R[0][1] - R[0][0] * R[2][1];
        R[2][2] = R[0][0] * R[1][1] - R[1][0] * R[0][1];
    }

    public static void computeJacobianEE(double[] q, boolean isRight, IKWorkspace ws) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                ws.T[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = ws.params;
        params[0][0] = 0;
        params[0][1] = L1 + L0;
        params[0][2] = 0;
        params[0][3] = -Math.PI / 2;
        params[0][4] = q[0];
        params[1][0] = -Math.PI / 2;
        params[1][1] = d2;
        params[1][2] = 0;
        params[1][3] = -Math.PI / 2;
        params[1][4] = q[1];
        params[2][0] = -Math.PI / 2;
        params[2][1] = 0;
        params[2][2] = 0;
        params[2][3] = -Math.PI;
        params[2][4] = q[2];
        params[3][0] = 0;
        params[3][1] = 0;
        params[3][2] = L4;
        params[3][3] = -Math.PI / 2;
        params[3][4] = q[3];
        params[4][0] = -Math.PI / 2;
        params[4][1] = L5 + L6;
        params[4][2] = 0;
        params[4][3] = 0;
        params[4][4] = q[4];
        params[5][0] = -Math.PI / 2;
        params[5][1] = 0;
        params[5][2] = 0;
        params[5][3] = 0;
        params[5][4] = q[5];

        for (int i = 0; i < NUM_JOINTS; i++) {
            double alpha = params[i][0];
            double a = params[i][2];
            double d = params[i][1];
            double offset = params[i][3];
            double qi = params[i][4];

            double ca = Math.cos(alpha), sa = Math.sin(alpha);
            ws.RxTx[0][0] = 1;
            ws.RxTx[0][1] = 0;
            ws.RxTx[0][2] = 0;
            ws.RxTx[0][3] = a;
            ws.RxTx[1][0] = 0;
            ws.RxTx[1][1] = ca;
            ws.RxTx[1][2] = -sa;
            ws.RxTx[1][3] = 0;
            ws.RxTx[2][0] = 0;
            ws.RxTx[2][1] = sa;
            ws.RxTx[2][2] = ca;
            ws.RxTx[2][3] = 0;
            ws.RxTx[3][0] = 0;
            ws.RxTx[3][1] = 0;
            ws.RxTx[3][2] = 0;
            ws.RxTx[3][3] = 1;

            multiply4x4(ws.T, ws.RxTx, ws.T_i_prime);

            ws.z0[i][0] = ws.T_i_prime[0][2];
            ws.z0[i][1] = ws.T_i_prime[1][2];
            ws.z0[i][2] = ws.T_i_prime[2][2];

            ws.p0[i][0] = ws.T_i_prime[0][3];
            ws.p0[i][1] = ws.T_i_prime[1][3];
            ws.p0[i][2] = ws.T_i_prime[2][3];

            double theta = qi + offset;
            double ct = Math.cos(theta), st = Math.sin(theta);
            ws.RzTz[0][0] = ct;
            ws.RzTz[0][1] = -st;
            ws.RzTz[0][2] = 0;
            ws.RzTz[0][3] = 0;
            ws.RzTz[1][0] = st;
            ws.RzTz[1][1] = ct;
            ws.RzTz[1][2] = 0;
            ws.RzTz[1][3] = 0;
            ws.RzTz[2][0] = 0;
            ws.RzTz[2][1] = 0;
            ws.RzTz[2][2] = 1;
            ws.RzTz[2][3] = d;
            ws.RzTz[3][0] = 0;
            ws.RzTz[3][1] = 0;
            ws.RzTz[3][2] = 0;
            ws.RzTz[3][3] = 1;

            multiply4x4(ws.T_i_prime, ws.RzTz, ws.T);
        }

        getToolMatrix(ws.MDH);
        multiply4x4(ws.T, ws.MDH, ws.T_tool);

        double p_tool_x = ws.T_tool[0][3];
        double p_tool_y = ws.T_tool[1][3];
        double p_tool_z = ws.T_tool[2][3];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ws.R_tool[i][j] = ws.T_tool[i][j];
            }
        }

        for (int i = 0; i < NUM_JOINTS; i++) {
            double dx = p_tool_x - ws.p0[i][0];
            double dy = p_tool_y - ws.p0[i][1];
            double dz = p_tool_z - ws.p0[i][2];

            ws.J0[0][i] = ws.z0[i][1] * dz - ws.z0[i][2] * dy;
            ws.J0[1][i] = ws.z0[i][2] * dx - ws.z0[i][0] * dz;
            ws.J0[2][i] = ws.z0[i][0] * dy - ws.z0[i][1] * dx;

            ws.J0[3][i] = ws.z0[i][0];
            ws.J0[4][i] = ws.z0[i][1];
            ws.J0[5][i] = ws.z0[i][2];
        }

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ws.RT[i][j] = ws.R_tool[j][i];
            }
        }

        for (int j = 0; j < NUM_JOINTS; j++) {
            ws.Je[0][j] = ws.RT[0][0] * ws.J0[0][j] + ws.RT[0][1] * ws.J0[1][j] + ws.RT[0][2] * ws.J0[2][j];
            ws.Je[1][j] = ws.RT[1][0] * ws.J0[0][j] + ws.RT[1][1] * ws.J0[1][j] + ws.RT[1][2] * ws.J0[2][j];
            ws.Je[2][j] = ws.RT[2][0] * ws.J0[0][j] + ws.RT[2][1] * ws.J0[1][j] + ws.RT[2][2] * ws.J0[2][j];

            ws.Je[3][j] = ws.RT[0][0] * ws.J0[3][j] + ws.RT[0][1] * ws.J0[4][j] + ws.RT[0][2] * ws.J0[5][j];
            ws.Je[4][j] = ws.RT[1][0] * ws.J0[3][j] + ws.RT[1][1] * ws.J0[4][j] + ws.RT[1][2] * ws.J0[5][j];
            ws.Je[5][j] = ws.RT[2][0] * ws.J0[3][j] + ws.RT[2][1] * ws.J0[4][j] + ws.RT[2][2] * ws.J0[5][j];
        }
    }

    public static void solveDLS(double[][] J, double[] e, double lambda, IKWorkspace ws) {
        int n = 6;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double sum = 0;
                for (int k = 0; k < NUM_JOINTS; k++) {
                    sum += J[i][k] * J[j][k];
                }
                ws.JJT[i][j] = sum;
            }
            ws.JJT[i][i] += lambda * lambda;
        }

        boolean success = solveLinearSystem(ws.JJT, e, ws.x, ws.M);
        if (!success) {
            for (int i = 0; i < NUM_JOINTS; i++)
                ws.dq[i] = 0.0;
            return;
        }

        for (int i = 0; i < NUM_JOINTS; i++) {
            double sum = 0;
            for (int k = 0; k < n; k++) {
                sum += J[k][i] * ws.x[k];
            }
            ws.dq[i] = sum;
        }
    }

    private static boolean solveLinearSystem(double[][] A, double[] b, double[] x, double[][] M) {
        int n = b.length;
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
                return false;
            }

            for (int k = i + 1; k < n; k++) {
                double factor = M[k][i] / M[i][i];
                for (int j = i; j <= n; j++) {
                    M[k][j] -= factor * M[i][j];
                }
            }
        }

        for (int i = n - 1; i >= 0; i--) {
            double sum = 0;
            for (int j = i + 1; j < n; j++) {
                sum += M[i][j] * x[j];
            }
            x[i] = (M[i][n] - sum) / M[i][i];
        }
        return true;
    }

    private static class IKWorkspace {
        final double[][] T = new double[4][4];
        final double[][] T_next = new double[4][4];
        final double[][] MDH = new double[4][4];
        final double[][] params = new double[6][5];

        final double[][] R0 = new double[3][3];
        final double[][] R1 = new double[3][3];
        final double[][] R0T = new double[3][3];
        final double[][] R = new double[3][3];
        final double[] dp = new double[3];
        final double[] dw = new double[3];
        final double[] delta = new double[6];

        final double[][] z0 = new double[6][3];
        final double[][] p0 = new double[6][3];
        final double[][] J0 = new double[6][6];
        final double[][] Je = new double[6][6];
        final double[][] RxTx = new double[4][4];
        final double[][] RzTz = new double[4][4];
        final double[][] T_i_prime = new double[4][4];
        final double[][] T_tool = new double[4][4];
        final double[][] R_tool = new double[3][3];
        final double[][] RT = new double[3][3];

        final double[][] JJT = new double[6][6];
        final double[] x = new double[6];
        final double[] dq = new double[6];
        final double[][] M = new double[6][7];
    }
}
