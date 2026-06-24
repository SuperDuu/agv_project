package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;

public class TestPointingIK {
    public static void main(String[] args) {
        // Target: [38.41, 32.65, 80.0]
        double px = 38.41, py = 32.65, pz = 80.0;
        
        System.out.println("=== Searching for 5-DOF (vertical tool) solutions with |q2| > 5 ===");
        
        // We will scan different tool yaw angles (from -180 to 180) 
        // to see if there is ANY yaw angle that allows q2 != 0 and Joint 4 negative.
        int found = 0;
        for (double yaw = -180; yaw <= 180; yaw += 2) {
            double yaw_rad = Math.toRadians(yaw);
            
            // Construct R_target with tool pointing straight down (alpha = 0), and rotated by yaw
            // R_target = Rz(yaw) * Ry(pi)
            // Ry(pi) points Z down: 
            // [ -1  0  0 ]
            // [  0  1  0 ]
            // [  0  0 -1 ]
            // Rz(yaw):
            // [ cy -sy  0 ]
            // [ sy  cy  0 ]
            // [  0   0  1 ]
            double cy = Math.cos(yaw_rad), sy = Math.sin(yaw_rad);
            double[][] R_target = {
                { -cy,  sy,  0 },
                { -sy, -cy,  0 },
                {   0,   0, -1 }
            };
            
            double[] q2_guesses = { 0.8, 0.4, -0.4, -0.8 };
            for (double q2val : q2_guesses) {
                double[] qHome = new double[NUM_JOINTS];
                // Joint 1 target azimuth
                qHome[0] = Math.max(Math.toRadians(-45), Math.min(Math.toRadians(45), Math.atan2(py, px)));
                qHome[1] = q2val;
                qHome[2] = 0.3; // Right arm Joint 3 positive
                qHome[3] = Math.toRadians(-45.0); // Joint 4 negative
                
                double[] q = solveIK(px, py, pz, R_target, qHome, true);
                if (q != null) {
                    boolean inLimits = true;
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        if (q[i] < JOINT_MIN_RIGHT[i] - 0.1 || q[i] > JOINT_MAX_RIGHT[i] + 0.1) {
                            inLimits = false;
                        }
                    }
                    if (inLimits && Math.abs(q[1]) > 5.0) {
                        System.out.printf("FOUND! Yaw=%.1f deg -> q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
                            yaw, q[0], q[1], q[2], q[3], q[4], q[5]);
                        found++;
                        if (found >= 20) return;
                    }
                }
            }
        }
        System.out.println("Search complete. Total solutions found: " + found);
    }
}
