package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;

public class TestRealIK {
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
        
        double[][] R3 = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R3[i][j] = R_target[i][j];
            }
        }
        
        // Target matrix
        double[][] T_target = {
            { R3[0][0], R3[0][1], R3[0][2], px },
            { R3[1][0], R3[1][1], R3[1][2], py },
            { R3[2][0], R3[2][1], R3[2][2], pz },
            { 0, 0, 0, 1 }
        };
        
        double[] q = {
            Math.toRadians(13.0),
            Math.toRadians(26.0),
            Math.toRadians(44.0),
            Math.toRadians(-38.0),
            Math.toRadians(0.0),
            Math.toRadians(0.0)
        };
        
        double[] minLimRad = new double[NUM_JOINTS];
        double[] maxLimRad = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            minLimRad[i] = Math.toRadians(JOINT_MIN_RIGHT[i]);
            maxLimRad[i] = Math.toRadians(JOINT_MAX_RIGHT[i]);
        }
        
        System.out.println("Running manual IK solver steps:");
        double alpha = 0.5;
        
        for (int iter = 0; iter < 100; iter++) {
            double[][] T_curr = computeFKMatrix(q, true);
            double[] e = computeTr2Delta(T_curr, T_target);
            
            double errNorm = 0;
            for (int i = 0; i < 6; i++) {
                errNorm += e[i] * e[i];
            }
            errNorm = Math.sqrt(errNorm);
            
            System.out.printf("Iter %d: errNorm = %.6f\n", iter, errNorm);
            System.out.printf("  q (deg): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
                Math.toDegrees(q[0]), Math.toDegrees(q[1]), Math.toDegrees(q[2]),
                Math.toDegrees(q[3]), Math.toDegrees(q[4]), Math.toDegrees(q[5]));
            
            if (errNorm < 1e-3) {
                System.out.println("CONVERGED!");
                return;
            }
            
            double[][] J = computeJacobianEE(q, true);
            double[] dq = solveDLS(J, e, 0.05);
            
            System.out.printf("  dq (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                Math.toDegrees(dq[0]), Math.toDegrees(dq[1]), Math.toDegrees(dq[2]),
                Math.toDegrees(dq[3]), Math.toDegrees(dq[4]), Math.toDegrees(dq[5]));
                
            for (int i = 0; i < NUM_JOINTS; i++) {
                q[i] = wrapToPi(q[i] + alpha * dq[i]);
                q[i] = Math.max(minLimRad[i], Math.min(maxLimRad[i], q[i]));
            }
        }
    }
}
