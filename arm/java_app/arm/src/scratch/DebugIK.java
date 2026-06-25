package scratch;

import gui.MainFrame;
import kinematics.Kinematics;
import java.util.Arrays;
import java.util.List;

public class DebugIK {
    public static void main(String[] args) {
        System.out.println("Initializing MainFrame...");
        MainFrame frame = new MainFrame();
        
        double px = 38.41, py = 32.65, pz = 80.0;
        boolean isRight = true;
        
        System.out.println("----------------- Calling tryAlpha directly -----------------");
        double alphaDeg = -60.0;
        
        double alpha_rad = Math.toRadians(alphaDeg);
        double q1_min = isRight ? Kinematics.JOINT_MIN_RIGHT[0] : Kinematics.JOINT_MIN_LEFT[0];
        double q1_max = isRight ? Kinematics.JOINT_MAX_RIGHT[0] : Kinematics.JOINT_MAX_LEFT[0];
        double q1_base = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), Math.atan2(py, px)));
        
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };

        double[] yawOffsets = { 0.0, -15.0, 15.0, -30.0, 30.0 };
        double[] activeAngles = isRight ? frame.getAnglesRight() : frame.getAnglesLeft();
        
        System.out.println("q1_base (deg): " + Math.toDegrees(q1_base));
        System.out.println("activeAngles: " + Arrays.toString(activeAngles));
        
        for (double offsetDeg : yawOffsets) {
            double yaw = q1_base + Math.toRadians(offsetDeg);
            double cy = Math.cos(yaw), sy = Math.sin(yaw);
            double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
            double[][] R_target = multiplyMatrices(R_z, R_y);

            double[] qInit = new double[Kinematics.NUM_JOINTS];
            for (int i = 0; i < Kinematics.NUM_JOINTS; i++) {
                qInit[i] = Math.toRadians(activeAngles[i]);
            }
            
            double[] q = Kinematics.solveIK(px, py, pz, R_target, qInit, isRight);
            System.out.printf("\n--- Yaw Offset: %.1f deg ---\n", offsetDeg);
            if (q == null) {
                System.out.println("solveIK returned NULL");
            } else {
                System.out.println("solveIK result (already deg!): " + Arrays.toString(q));
                
                boolean inLimits = isWithinLimits(q, isRight);
                System.out.println("isWithinLimits: " + inLimits);
                
                if (!inLimits) {
                    double[] minLim = isRight ? Kinematics.JOINT_MIN_RIGHT : Kinematics.JOINT_MIN_LEFT;
                    double[] maxLim = isRight ? Kinematics.JOINT_MAX_RIGHT : Kinematics.JOINT_MAX_LEFT;
                    for (int i = 0; i < Kinematics.NUM_JOINTS; i++) {
                        double valDeg = q[i];
                        if (valDeg < minLim[i] - 0.1 || valDeg > maxLim[i] + 0.1) {
                            System.out.printf("  Joint %d: %.2f out of limit [%.1f, %.1f]\n", 
                                i+1, valDeg, minLim[i], maxLim[i]);
                        }
                    }
                }
                
                double err = computePositionError(q, px, py, pz);
                System.out.printf("Position Error: %.4f mm\n", err);
            }
        }
        
        System.exit(0);
    }
    
    private static double[][] multiplyMatrices(double[][] A, double[][] B) {
        int n = A.length;
        double[][] C = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                C[i][j] = 0;
                for (int k = 0; k < n; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }
    
    private static boolean isWithinLimits(double[] q, boolean isRight) {
        double[] minLim = isRight ? Kinematics.JOINT_MIN_RIGHT : Kinematics.JOINT_MIN_LEFT;
        double[] maxLim = isRight ? Kinematics.JOINT_MAX_RIGHT : Kinematics.JOINT_MAX_LEFT;
        for (int i = 0; i < q.length; i++) {
            if (q[i] < (minLim[i] - 0.1) || q[i] > (maxLim[i] + 0.1))
                return false;
        }
        return true;
    }
    
    private static double computePositionError(double[] qDeg, double tx, double ty, double tz) {
        double[] qRad = new double[Kinematics.NUM_JOINTS];
        for (int i = 0; i < Kinematics.NUM_JOINTS; i++) {
            qRad[i] = Math.toRadians(qDeg[i]);
        }
        double[][] T = Kinematics.computeFKMatrix(qRad, true);
        double dx = T[0][3] - tx;
        double dy = T[1][3] - ty;
        double dz = T[2][3] - tz;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }
}
