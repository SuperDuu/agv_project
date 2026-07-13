public class FindSafeTransition {
    public static void main(String[] args) {
        double[] qEnd = { 0.0, 45.0, 86.0, -95.0, 57.0, 60.0 };
        double lowChairHeight = 102.4;
        double[] lowChairCenter = { 39.88, 25.63, 0.0 };
        double margin = 3.0;
        double minX = lowChairCenter[0] - 14.0 - margin;
        double maxX = lowChairCenter[0] + 14.0 + margin;
        double minY = lowChairCenter[1] - 14.0 - margin;
        double maxY = lowChairCenter[1] + 14.0 + margin;
        double minZ = 0.0 - margin;
        double maxZ = lowChairHeight + margin;

        System.out.println("Searching for safe Joint 3 starting angle:");
        for (double q3Start = 20.0; q3Start <= 165.0; q3Start += 1.0) {
            double[] qStart = { 0.0, -90.0, q3Start, -95.0, 57.0, 60.0 };
            if (isWithinLimits(qStart)) {
                boolean allOk = true;
                for (int step = 0; step <= 8; step++) {
                    double t = step / 8.0;
                    double[] q = new double[6];
                    for (int i = 0; i < 6; i++) {
                        q[i] = qStart[i] + (qEnd[i] - qStart[i]) * t;
                    }
                    double[][] pts = computeAllJoints3DForAngles(q, true);
                    double[] pt6 = pts[6];
                    double[] pt7 = pts[7];
                    double[] ptMid = { (pt6[0] + pt7[0])/2.0, (pt6[1] + pt7[1])/2.0, (pt6[2] + pt7[2])/2.0 };
                    
                    double[][] checkPts = { pt6, pt7, ptMid };
                    for (double[] pt : checkPts) {
                        if (pt[0] >= minX && pt[0] <= maxX
                                && pt[1] >= minY && pt[1] <= maxY
                                && pt[2] >= minZ && pt[2] <= maxZ) {
                            allOk = false;
                            break;
                        }
                    }
                    if (!allOk) break;
                }
                if (allOk) {
                    System.out.printf("Found safe Joint 3 start: %.1f\n", q3Start);
                }
            }
        }
    }

    private static boolean isWithinLimits(double[] q) {
        double[] minLim = { -45, -90, 20, -95, -90, -60 };
        double[] maxLim = { 45, 90, 165, -15, 90, 60 };
        for (int i = 0; i < q.length; i++) {
            if (q[i] < (minLim[i] - 0.1) || q[i] > (maxLim[i] + 0.1))
                return false;
        }
        return true;
    }

    private static double[][] computeAllJoints3DForAngles(double[] anglesDeg, boolean isRight) {
        double[][] pts = new double[8][3];
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        pts[0] = new double[] { 0, 0, 0 };
        double L0 = 130.0;
        double L1 = 0.0;
        double L2 = 32.0;
        double L3 = 0.0;
        double L4 = 20.0;
        double L5 = 25.0;
        double L6 = 0.0;
        double L7 = 15.0;
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(anglesDeg[0]) },
                { -Math.PI / 2, d2, 0, -Math.PI / 2, Math.toRadians(anglesDeg[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(anglesDeg[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(anglesDeg[3]) },
                { -Math.PI / 2, L5 + L6, 0, 0, Math.toRadians(anglesDeg[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(anglesDeg[5]) }
        };
        for (int i = 0; i < 6; i++) {
            T = multiply4x4(T, getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
            pts[i + 1] = new double[] { T[0][3], T[1][3], T[2][3] };
        }
        T = multiply4x4(T, getToolMatrix());
        pts[7] = new double[] { T[0][3], T[1][3], T[2][3] };
        return pts;
    }

    private static double[][] getMDHMatrix(double alpha, double d, double a, double theta_offset, double theta) {
        double ct = Math.cos(theta + theta_offset);
        double st = Math.sin(theta + theta_offset);
        double ca = Math.cos(alpha);
        double sa = Math.sin(alpha);
        return new double[][] {
                { ct, -st, 0, a },
                { st * ca, ct * ca, -sa, -sa * d },
                { st * sa, ct * sa, ca, ca * d },
                { 0, 0, 0, 1 }
        };
    }

    private static double[][] getToolMatrix() {
        double L7 = 15.0;
        return new double[][] {
                { 0, -1, 0, 0 },
                { 0, 0, -1, -L7 },
                { 1, 0, 0, 0 },
                { 0, 0, 0, 1 }
        };
    }

    private static double[][] multiply4x4(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }
}
