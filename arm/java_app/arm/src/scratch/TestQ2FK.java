package scratch;

public class TestQ2FK {
    public static void main(String[] args) {
        double tx = 38.41, ty = 32.65, tz = 80.00;
        System.out.println("Searching for solutions close to target (" + tx + ", " + ty + ", " + tz + ") within 5.0 mm:");
        System.out.println("q1\tq2\tq3\tq4\tEE_X\tEE_Y\tEE_Z\tDist");
        
        // Scan Joint 1 from -45 to 45
        for (double q1 = -45; q1 <= 45; q1 += 2) {
            // Scan Joint 2 from -90 to 90
            for (double q2 = -90; q2 <= 90; q2 += 2) {
                // Scan Joint 3 from -90 to 90
                for (double q3 = -90; q3 <= 90; q3 += 2) {
                    // Scan Joint 4 (Right Arm limits: -140 to -30)
                    for (double q4 = -140; q4 <= -30; q4 += 2) {
                        double[] q = { q1, q2, q3, q4, 0, 0 };
                        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
                        double d2 = 20.0; // L2+L3
                        double[][] params = {
                            { 0, 125.0, 0, -Math.PI / 2, Math.toRadians(q[0]) },
                            { -Math.PI / 2, d2, 0, -Math.PI / 2, Math.toRadians(q[1]) },
                            { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(q[2]) },
                            { 0, 0, 20.0, -Math.PI / 2, Math.toRadians(q[3]) },
                            { -Math.PI / 2, 30.0, 0, -Math.PI / 2, Math.toRadians(q[4]) },
                            { -Math.PI / 2, 0, 0, 0, Math.toRadians(q[5]) }
                        };
                        for (int i = 0; i < 6; i++) {
                            T = TestFK.multiply4x4(T, TestFK.getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
                        }
                        // tool matrix
                        double[][] tool = {
                            { 1, 0, 0, 0 },
                            { 0, 0, -1, 0 },
                            { 0, 1, 0, 10.0 },
                            { 0, 0, 0, 1 }
                        };
                        T = TestFK.multiply4x4(T, tool);
                        
                        double dx = T[0][3] - tx;
                        double dy = T[1][3] - ty;
                        double dz = T[2][3] - tz;
                        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                        
                        if (dist < 1.5) {
                            System.out.printf("%+.1f\t%+.1f\t%+.1f\t%+.1f\t%.2f\t%.2f\t%.2f\t%.3f\n",
                                q1, q2, q3, q4, T[0][3], T[1][3], T[2][3], dist);
                        }
                    }
                }
            }
        }
    }
}
