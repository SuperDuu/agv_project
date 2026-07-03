import kinematics.Kinematics;
import java.util.List;
import java.util.ArrayList;

public class ScanIK {
    public static void main(String[] args) {
        double px = 33.79;
        double py = 28.44;
        double pz = 100.00;
        boolean isRight = true;
        
        double[] JOINT_MIN_RIGHT = { -45, -90, -90, -140, -90, -90 };
        double[] JOINT_MAX_RIGHT = { 45, 90, 90, -30, 90, 90 };
        
        double q1_base = Math.atan2(py, px);
        double[] qInit = {
            Math.toRadians(0.0),
            Math.toRadians(0.0),
            Math.toRadians(10.0),
            Math.toRadians(-30.0),
            Math.toRadians(0.0),
            Math.toRadians(0.0)
        };
        
        double bestErr = Double.MAX_VALUE;
        double bestAlpha = 0;
        double bestYawOff = 0;
        double[] bestQ = null;
        boolean foundAny = false;
        
        for (double alpha = -90; alpha <= 30; alpha += 1.0) {
            double alpha_rad = Math.toRadians(alpha);
            double ca = Math.cos(Math.PI + alpha_rad);
            double sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            
            for (double yawOff = -90; yawOff <= 90; yawOff += 1.0) {
                double yaw = q1_base + Math.toRadians(yawOff);
                double cy = Math.cos(yaw);
                double sy = Math.sin(yaw);
                double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                double[][] R_target = kinematics.Kinematics.multiplyMatrices(R_z, R_y);
                
                double[] q = kinematics.Kinematics.solveIK(px, py, pz, R_target, qInit, isRight);
                if (q != null) {
                    boolean ok = true;
                    for (int i = 0; i < 6; i++) {
                        if (q[i] < JOINT_MIN_RIGHT[i] - 0.1 || q[i] > JOINT_MAX_RIGHT[i] + 0.1) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) {
                        // Convert q back to radians for Kinematics.computeFKMatrix
                        double[] qRad = new double[6];
                        for(int i=0; i<6; i++) qRad[i] = Math.toRadians(q[i]);
                        double[][] T_best = kinematics.Kinematics.computeFKMatrix(qRad, isRight);
                        double dx = T_best[0][3] - px;
                        double dy = T_best[1][3] - py;
                        double dz = T_best[2][3] - pz;
                        double err = Math.sqrt(dx*dx + dy*dy + dz*dz);
                        if (err < bestErr) {
                            bestErr = err;
                            bestAlpha = alpha;
                            bestYawOff = yawOff;
                            bestQ = q.clone();
                        }
                        if (err < 0.15) {
                            System.out.printf("FOUND: alpha=%.1f, yawOff=%.1f, err=%.4f cm\n", alpha, yawOff, err);
                            System.out.printf("  q = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", q[0], q[1], q[2], q[3], q[4], q[5]);
                            foundAny = true;
                        }
                    }
                }
            }
        }
        
        if (!foundAny && bestQ != null) {
            System.out.printf("Best: alpha=%.1f, yawOff=%.1f, err=%.4f cm\n", bestAlpha, bestYawOff, bestErr);
            System.out.printf("  q = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", bestQ[0], bestQ[1], bestQ[2], bestQ[3], bestQ[4], bestQ[5]);
        }
    }
}
