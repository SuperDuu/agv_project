package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;

public class TestOffAxis {
    public static void main(String[] args) {
        // Target is at X = 15, Y = 30, Z = 80 (azimuth = 63.4 degrees, which is > 45 limit of Joint 1)
        double px = 15.0, py = 30.0, pz = 80.0;
        
        System.out.println("=== Testing off-axis target (15, 30, 80) ===");
        
        double[] alphas = { -60, -30, 0, 30, 60 };
        double[] q2_guesses = { 0.8, 0.4, 0.0, -0.4, -0.8 };
        
        for (double alpha : alphas) {
            System.out.println("\n--- Alpha = " + alpha + " deg ---");
            
            double alpha_rad = Math.toRadians(alpha);
            double q1 = Math.atan2(py, px);
            // Clamp to [-45, 45]
            q1 = Math.max(Math.toRadians(-45), Math.min(Math.toRadians(45), q1));
            double c1 = Math.cos(q1), s1 = Math.sin(q1);
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            double[][] R_z = { { c1, -s1, 0 }, { s1, c1, 0 }, { 0, 0, 1 } };
            double[][] R_target = multiplyMatrices(R_z, R_y);
            
            for (double q2val : q2_guesses) {
                double[] qHome = new double[NUM_JOINTS];
                qHome[0] = q1;
                qHome[1] = q2val;
                qHome[2] = 0.3; // Right arm Joint 3 positive
                qHome[3] = Math.toRadians(-45.0); // Joint 4 negative
                
                double[] q = solveIK(px, py, pz, R_target, qHome, true);
                if (q != null) {
                    boolean inLimits = true;
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        if (q[i] < JOINT_MIN_RIGHT[i] - 0.1 || q[i] > JOINT_MAX_RIGHT[i] + 0.1) {
                            inLimits = false;
                            break;
                        }
                    }
                    if (inLimits) {
                        System.out.printf("  q2init=%+5.1f -> q=[%+6.1f, %+6.1f, %+6.1f, %+7.1f, %+6.1f, %+6.1f] OK\n",
                            Math.toDegrees(q2val), q[0], q[1], q[2], q[3], q[4], q[5]);
                    } else {
                        System.out.printf("  q2init=%+5.1f -> q=[%+6.1f, %+6.1f, %+6.1f, %+7.1f, %+6.1f, %+6.1f] OUT_OF_LIMITS\n",
                            Math.toDegrees(q2val), q[0], q[1], q[2], q[3], q[4], q[5]);
                    }
                } else {
                    System.out.printf("  q2init=%+5.1f -> FAILED\n", Math.toDegrees(q2val));
                }
            }
        }
    }
}
