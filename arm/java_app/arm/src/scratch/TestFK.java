package scratch;

public class TestFK {
    public static final double L0 = 120.0;
    public static final double L1 = 5.0;
    public static final double L2 = 10.0;
    public static final double L3 = 10.0;
    public static final double L4 = 20.0;
    public static final double L5 = 20.0;
    public static final double L6 = 10.0;
    public static final double L7 = 10.0;

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

    public static double[][] multiply4x4(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++)
                for (int j = 0; j < 4; j++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }

    public static double[][] computeFK(double[] q, boolean isRight) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = {
            { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(q[0]) },
            { -Math.PI / 2, d2, 0, -Math.PI / 2, Math.toRadians(q[1]) },
            { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(q[2]) },
            { 0, 0, L4, -Math.PI / 2, Math.toRadians(q[3]) },
            { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(q[4]) },
            { -Math.PI / 2, 0, 0, 0, Math.toRadians(q[5]) }
        };
        double[][] pts = new double[8][3];
        pts[0] = new double[] { 0, 0, 0 };
        for (int i = 0; i < 6; i++) {
            T = multiply4x4(T, getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
            pts[i + 1] = new double[] { T[0][3], T[1][3], T[2][3] };
        }
        return pts;
    }

    public static void main(String[] args) {
        System.out.println("Right Arm (q2=0, q3=0, q4=-90):");
        printPoints(computeFK(new double[]{ 0, 0, 0, -90, 0, 0 }, true));
        
        System.out.println("\nRight Arm (q2=20, q3=0, q4=-90):");
        printPoints(computeFK(new double[]{ 0, 20, 0, -90, 0, 0 }, true));

        System.out.println("\nRight Arm (q2=-20, q3=0, q4=-90):");
        printPoints(computeFK(new double[]{ 0, -20, 0, -90, 0, 0 }, true));
    }

    private static void printPoints(double[][] pts) {
        System.out.printf("  Shoulder (pts[3]): [%.2f, %.2f, %.2f]\n", pts[3][0], pts[3][1], pts[3][2]);
        System.out.printf("  Elbow    (pts[4]): [%.2f, %.2f, %.2f]\n", pts[4][0], pts[4][1], pts[4][2]);
        System.out.printf("  Wrist    (pts[5]): [%.2f, %.2f, %.2f]\n", pts[5][0], pts[5][1], pts[5][2]);
    }
}
