public class TestWaypointHeight {
    public static void main(String[] args) {
        double sharedQ1 = 0.0;
        
        System.out.println("Searching for a valid transferHighRight:");
        for (double q2 = -90.0; q2 <= -20.0; q2 += 5.0) {
            for (double q3 = 60.0; q3 <= 140.0; q3 += 5.0) {
                for (double q4 = -95.0; q4 <= -15.0; q4 += 5.0) {
                    double[] q = { sharedQ1, q2, q3, q4, 20.0, -25.0 };
                    if (isWithinLimits(q)) {
                        double[][] pts = computeAllJoints3DForAngles(q, true);
                        double z6 = pts[6][2];
                        double z7 = pts[7][2];
                        double zMid = (pts[6][2] + pts[7][2]) / 2.0;
                        
                        // We want all gripper points to be above 128.0 cm!
                        if (z6 > 128.0 && z7 > 128.0 && zMid > 128.0) {
                            // Also check if the position is not too far away
                            if (pts[7][0] < 65.0) {
                                System.out.printf("q2=%.1f q3=%.1f q4=%.1f -> Joint6_Z=%.2f, TCP_Z=%.2f, TCP_X=%.2f\n",
                                        q2, q3, q4, z6, z7, pts[7][0]);
                            }
                        }
                    }
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
