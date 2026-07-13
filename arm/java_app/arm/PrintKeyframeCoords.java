import kinematics.RobotTransmission;

public class PrintKeyframeCoords {
    public static void main(String[] args) {
        double sharedQ1 = 0.0;
        
        double[] homeRight = { sharedQ1, 0, 20, -35, 0, 0 };
        double[] foldedHomeRight = { sharedQ1, 0.0, 120.0, -90.0, 0.0, -52.0 };
        
        // Option B (Low Chair Y = -26.92)
        double[] lowPickRightB = { sharedQ1, -48.0, 48.0, -51.0, -90.0, -54.0 };
        double[] lowHoverRightB = { sharedQ1, -54.0, 54.0, -59.0, -90.0, -60.0 };
        
        // Option C (Low Chair Y = 4.80)
        double[] lowPickRightC = { sharedQ1, 20.0, 34.0, 15.0, -90.0, 40.0 };
        double[] lowHoverRightC = { sharedQ1, 26.0, 36.0, 19.0, -90.0, 52.0 };
        
        double[] highPlaceRight = { sharedQ1, -10.0, 134.0, -93.0, -90.0, -24.0 };
        double[] highHoverRight = { sharedQ1, -10.0, 142.0, -95.0, -90.0, -24.0 };

        System.out.println("=== Trajectory Keyframe TCP Coordinates ===");
        printTCP("homeRight", homeRight);
        printTCP("foldedHomeRight", foldedHomeRight);
        printTCP("highPlaceRight", highPlaceRight);
        printTCP("highHoverRight", highHoverRight);
        System.out.println("--- Option B (Separated Y) ---");
        printTCP("lowPickRightB", lowPickRightB);
        printTCP("lowHoverRightB", lowHoverRightB);
        System.out.println("--- Option C (Aligned Y) ---");
        printTCP("lowPickRightC", lowPickRightC);
        printTCP("lowHoverRightC", lowHoverRightC);
    }

    private static void printTCP(String name, double[] q) {
        double[][] T = computeMDHMatrix(q, true);
        System.out.printf("  %-18s: TCP = (%.2f, %.2f, %.2f)\n", name, T[0][3], T[1][3], T[2][3]);
    }

    private static double[][] computeMDHMatrix(double[] anglesDeg, boolean isRight) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double L0 = 130.0;
        double L1 = 0.0;
        double L2 = 32.0;
        double L3 = 0.0;
        double L4 = 20.0;
        double L5 = 25.0;
        double L6 = 0.0;
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
        }
        T = multiply4x4(T, getToolMatrix());
        return T;
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
