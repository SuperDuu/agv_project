package scratch;

public class TestWristSearch {
    public static void main(String[] args) {
        double px = 38.41, py = 32.65, pz = 80.0;
        double alpha_rad = Math.toRadians(-60);
        double q1_target = Math.atan2(py, px);
        double c1 = Math.cos(q1_target), s1 = Math.sin(q1_target);
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_target = TestFK.multiply4x4(
            new double[][]{ {c1,-s1,0,0},{s1,c1,0,0},{0,0,1,0},{0,0,0,1} },
            new double[][]{ {ca,0,sa,0},{0,1,0,0},{-sa,0,ca,0},{0,0,0,1} }
        );
        
        double tx = R_target[0][2];
        double ty = R_target[1][2];
        double tz = R_target[2][2];
        
        double wx = px - 10.0 * tx;
        double wy = py - 10.0 * ty;
        double wz = pz - 10.0 * tz;
        
        System.out.printf("Target Wrist Center: [%.2f, %.2f, %.2f]\n", wx, wy, wz);
        System.out.println("Solutions within 2.0 mm limit:");
        System.out.println("q1\tq2\tq3\tq4\tDist");
        
        int step = 2;
        for (double q1 = -45; q1 <= 45; q1 += step) {
            for (double q2 = -90; q2 <= 90; q2 += step) {
                for (double q3 = -90; q3 <= 90; q3 += step) {
                    for (double q4 = -140; q4 <= -30; q4 += step) {
                        double[] q = { q1, q2, q3, q4, 0, 0 };
                        double[][] pts = TestFK.computeFK(q, true);
                        double dx = pts[5][0] - wx;
                        double dy = pts[5][1] - wy;
                        double dz = pts[5][2] - wz;
                        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                        
                        if (dist < 2.0) {
                            System.out.printf("%+.1f\t%+.1f\t%+.1f\t%+.1f\t%.3f\n",
                                q1, q2, q3, q4, dist);
                        }
                    }
                }
            }
        }
    }
}
