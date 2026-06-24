package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;

public class TestIKSearch {
    public static void main(String[] args) {
        // Let's test a grid of target positions (X from 10 to 60, Y from -30 to 30, Z from 50 to 120)
        // and see if we can find any valid IK solutions with:
        // 1. |Joint 2| > 5 degrees
        // 2. Joint 4 in [-140, -30] (strictly negative)
        
        System.out.println("Searching for valid configurations with |q2| > 5 and q4 < -30...");
        int count = 0;
        
        for (double px = 15; px <= 65; px += 10) {
            for (double py = -30; py <= 30; py += 10) {
                for (double pz = 50; pz <= 120; pz += 10) {
                    // Try different alpha values
                    for (double alpha = -60; alpha <= 60; alpha += 30) {
                        double alpha_rad = Math.toRadians(alpha);
                        double q1 = Math.atan2(py, px);
                        q1 = Math.max(Math.toRadians(-45), Math.min(Math.toRadians(45), q1));
                        double c1 = Math.cos(q1), s1 = Math.sin(q1);
                        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
                        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
                        double[][] R_z = { { c1, -s1, 0 }, { s1, c1, 0 }, { 0, 0, 1 } };
                        double[][] R_target = multiplyMatrices(R_z, R_y);
                        
                        // Try different q2 guesses
                        double[] q2_guesses = { 0.8, 0.4, -0.4, -0.8 };
                        for (double q2init : q2_guesses) {
                            double[] qHome = new double[NUM_JOINTS];
                            qHome[0] = q1;
                            qHome[1] = q2init;
                            qHome[2] = 0.3; // Right arm Joint 3 positive
                            qHome[3] = Math.toRadians(-45.0); // Joint 4 negative
                            
                            double[] q = solveIK(px, py, pz, R_target, qHome, true);
                            if (q != null) {
                                // check limits
                                boolean inLimits = true;
                                for (int i = 0; i < NUM_JOINTS; i++) {
                                    if (q[i] < JOINT_MIN_RIGHT[i] - 0.1 || q[i] > JOINT_MAX_RIGHT[i] + 0.1) {
                                        inLimits = false;
                                        break;
                                    }
                                }
                                if (inLimits && Math.abs(q[1]) > 5.0) {
                                    System.out.printf("FOUND! Target=[%.1f, %.1f, %.1f] alpha=%.1f -> q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
                                        px, py, pz, alpha, q[0], q[1], q[2], q[3], q[4], q[5]);
                                    count++;
                                    if (count >= 20) return; // Print first 20 solutions
                                }
                            }
                        }
                    }
                }
            }
        }
        System.out.println("Done searching. Total solutions found: " + count);
    }
}
