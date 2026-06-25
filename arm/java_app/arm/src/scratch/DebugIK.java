package scratch;

import kinematics.Kinematics;

public class DebugIK {
    public static void main(String[] args) {
        double px = -33.83, py = 6.11, pz = 80.00;
        
        // 1. Calculate q1_base
        double q1_min = Kinematics.JOINT_MIN_LEFT[0];
        double q1_max = Kinematics.JOINT_MAX_LEFT[0];
        double q1_base = -Math.atan2(py, -px);
        q1_base = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), q1_base));
        
        System.out.printf("q1_base (deg): %.4f\n", Math.toDegrees(q1_base));
        
        double yaw = q1_base; // offset = 0
        double yawR = -yaw;
        double cyR = Math.cos(yawR), syR = Math.sin(yawR);
        double[][] R_z_right = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
        
        double alphaDeg = -60.0; // let's try alphaDeg = -60.0 first, then we can search
        double alpha_rad = Math.toRadians(alphaDeg);
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        
        double[][] R_target_right = Kinematics.multiplyMatrices(R_z_right, R_y);
        double[][] R_target_left = {
            {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
            { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
            { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
        };
        
        double[][] T_target = {
            { R_target_left[0][0], R_target_left[0][1], R_target_left[0][2], px },
            { R_target_left[1][0], R_target_left[1][1], R_target_left[1][2], py },
            { R_target_left[2][0], R_target_left[2][1], R_target_left[2][2], pz },
            { 0, 0, 0, 1 }
        };
        
        // Initial guess: default pose of left arm
        double[] q = { 0, 0, Math.toRadians(-10), Math.toRadians(30), 0, 0 };
        
        double[] minLimRad = new double[6];
        double[] maxLimRad = new double[6];
        for (int i = 0; i < 6; i++) {
            minLimRad[i] = Math.toRadians(Kinematics.JOINT_MIN_LEFT[i]);
            maxLimRad[i] = Math.toRadians(Kinematics.JOINT_MAX_LEFT[i]);
        }
        
        System.out.println("Running solveIK loop for LEFT arm...");
        for (int iter = 0; iter < 100; iter++) {
            double[][] T_curr = Kinematics.computeFKMatrix(q, false);
            double[] e = Kinematics.computeTr2Delta(T_curr, T_target);
            
            double errNorm = 0;
            for (int i = 0; i < 6; i++) errNorm += e[i] * e[i];
            errNorm = Math.sqrt(errNorm);
            
            if (errNorm < 1e-3) {
                System.out.printf("CONVERGED at iter %d! q (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", 
                    iter, Math.toDegrees(q[0]), Math.toDegrees(q[1]), Math.toDegrees(q[2]),
                    Math.toDegrees(q[3]), Math.toDegrees(q[4]), Math.toDegrees(q[5]));
                System.exit(0);
            }
            
            double[][] J = Kinematics.computeJacobianEE(q, false);
            double[] dq = Kinematics.solveDLS(J, e, 0.05);
            
            for (int i = 0; i < 6; i++) {
                q[i] = Kinematics.wrapToPi(q[i] + 0.5 * dq[i]);
                q[i] = Math.max(minLimRad[i], Math.min(maxLimRad[i], q[i]));
            }
        }
        System.out.println("FAILED to converge at alpha = -60.0");
        System.exit(0);
    }
}
