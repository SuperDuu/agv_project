package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;
import java.util.*;

/**
 * Exact simulation of the ORIGINAL tryAlpha with early-exit bug.
 * Tests if the early-exit at preferredYaw=45 causes the solver to miss yaw=-60.
 */
public class DiagTrajIK3 {
    
    public static void main(String[] args) {
        double px = 38.31, py = -22.06, pz = 100.00;
        boolean isRight = true;
        double[] qRef = {14.36, -51.21, 87.68, -100.69, -90.00, 1.40};
        double preferredYaw = 44.8;
        double alpha = -57.0; // The alpha that gives best result
        
        System.out.println("=== TESTING EARLY-EXIT BUG AT alpha=-57 ===");
        
        double q1_base = Math.atan2(py, px);
        double alpha_rad = Math.toRadians(alpha);
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        
        Double[] yawOffsets = { 0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -75.0, 75.0, -90.0, 90.0 };
        
        // Sort by distance from preferredYaw=44.8
        Arrays.sort(yawOffsets, (a, b) -> Double.compare(Math.abs(a - preferredYaw), Math.abs(b - preferredYaw)));
        
        System.out.println("Yaw offset scan order (sorted by distance from preferredYaw=" + preferredYaw + "):");
        for (int idx = 0; idx < yawOffsets.length; idx++) {
            System.out.printf("  [%d] offset=%.1f (dist=%.1f from pref)\n", idx, yawOffsets[idx], Math.abs(yawOffsets[idx] - preferredYaw));
        }
        
        System.out.println("\nScanning with ORIGINAL early-exit logic:");
        for (double offsetDeg : yawOffsets) {
            double yaw = q1_base + Math.toRadians(offsetDeg);
            double cy = Math.cos(yaw), sy = Math.sin(yaw);
            double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
            double[][] R_target = multiplyMatrices(R_z, R_y);
            
            double[] qInit = new double[NUM_JOINTS];
            for (int i = 0; i < NUM_JOINTS; i++) qInit[i] = Math.toRadians(qRef[i]);
            
            double[] q = solveIK(px, py, pz, R_target, qInit, isRight);
            boolean valid = q != null && isWithinLimits(q, isRight);
            double err = valid ? computePosErr(q, px, py, pz, isRight) : Double.MAX_VALUE;
            
            System.out.printf("  offset=%6.1f → valid=%s err=%8.4f mm",
                offsetDeg, valid, err * 10.0);
            
            // ORIGINAL early-exit check
            if (valid && Math.abs(offsetDeg - preferredYaw) < 1.0) {
                System.out.printf(" *** EARLY EXIT HERE (preferredYaw match) ***");
                System.out.printf("\n  RESULT: Would return with err=%.4f mm. ", err * 10.0);
                if (err * 10.0 < 1.0) {
                    System.out.println("This IS strict. Early exit is FINE here.");
                } else {
                    System.out.println("This is NOT strict! Early exit is BAD here!");
                }
                return;  // Simulate the early exit
            }
            
            if (valid && err < 0.1) {
                System.out.printf(" *** EARLY EXIT (err<0.1, offset=0) ***");
                if (offsetDeg == 0.0) {
                    System.out.printf("\n  RESULT: Would return with err=%.4f mm\n", err * 10.0);
                    return;
                }
            }
            
            System.out.println();
        }
        System.out.println("  Completed full scan without early exit.");
    }
    
    static double computePosErr(double[] qDeg, double tx, double ty, double tz, boolean isRight) {
        double[] qRad = new double[6];
        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(qDeg[i]);
        double[][] T = computeFKMatrix(qRad, isRight);
        double dx = T[0][3] - tx, dy = T[1][3] - ty, dz = T[2][3] - tz;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    static boolean isWithinLimits(double[] qDeg, boolean isRight) {
        double[] minLim = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (qDeg[i] < minLim[i] - 0.5 || qDeg[i] > maxLim[i] + 0.5) return false;
        }
        return true;
    }
}
