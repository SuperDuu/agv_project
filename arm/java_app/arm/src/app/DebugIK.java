package app;

import kinematics.Kinematics;

public class DebugIK {
    public static void main(String[] args) {
        System.out.println("=== ROBUST GRIPPER ORIENTATION TEST AND DEBUG ===");
        
        // Simulating a line trajectory from (30, -10, 60) to (30, 20, 60)
        double sx = 9.79, sy = -2.0, sz = 67.71;
        double ex = 9.79, ey = 2.0, ez = 67.71;
        int steps = 5;
        
        System.out.println("\n--- MODE: Bàn tay song song mặt đất (alpha = 0.0) ---");
        boolean isRight = true;
        
        double[] qInit = { 0, 0, 10, -30, 0, 0 }; // default starting angles
        double[] qPrev = new double[6];
        for(int i=0; i<6; i++) qPrev[i] = Math.toRadians(qInit[i]);
        
        for (int i = 0; i <= steps; i++) {
            double r = i / (double) steps;
            double tx = sx + (ex - sx) * r;
            double ty = sy + (ey - sy) * r;
            double tz = sz + (ez - sz) * r;
            
            // Under fixedGround = true, alpha is forced to 0.0
            double alphaDeg = 0.0;
            double alpha_rad = Math.toRadians(alphaDeg);
            double q1_base = Math.atan2(ty, tx);
            
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            double cy = Math.cos(q1_base), syAngle = Math.sin(q1_base);
            double[][] R_z = { { cy, -syAngle, 0 }, { syAngle, cy, 0 }, { 0, 0, 1 } };
            double[][] R_target = multiplyMatrices(R_z, R_y);
            
            double[] qSol = Kinematics.solveIK(tx, ty, tz, R_target, qPrev, isRight);
            if (qSol != null) {
                // FK
                double[] qRad = new double[6];
                for(int j=0; j<6; j++) qRad[j] = Math.toRadians(qSol[j]);
                double[][] T_fk = Kinematics.computeFKMatrix(qRad, isRight);
                double[][] R_fk = extractRotation(T_fk);
                
                // Position error
                double err = Math.sqrt(Math.pow(T_fk[0][3] - tx, 2) + Math.pow(T_fk[1][3] - ty, 2) + Math.pow(T_fk[2][3] - tz, 2));
                
                // Tool axes
                double[] uz = { R_fk[0][2], R_fk[1][2], R_fk[2][2] };
                
                System.out.printf("Point %d: Target=(%.2f, %.2f, %.2f) | Solved q=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] | PosErr=%.4f mm | Tool Z-axis=[%.4f, %.4f, %.4f]\n",
                    i, tx, ty, tz, qSol[0], qSol[1], qSol[2], qSol[3], qSol[4], qSol[5], err, uz[0], uz[1], uz[2]);
                
                // Update previous state for continuity
                qPrev = qRad;
            } else {
                System.out.printf("Point %d: Target=(%.2f, %.2f, %.2f) | IK FAILED!\n", i, tx, ty, tz);
            }
        }
    }

    private static double[][] multiplyMatrices(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
            }
        }
        return C;
    }

    private static double[][] extractRotation(double[][] T) {
        double[][] R = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = T[i][j];
            }
        }
        return R;
    }
}
