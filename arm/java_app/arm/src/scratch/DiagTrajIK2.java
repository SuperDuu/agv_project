package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;
import java.util.*;

/**
 * Simulates the EXACT trajectory solver code path for the failing Point 5.
 * Previous solution (Point 4): q = [14.36, -51.21, 87.68, -100.69, -90.00, 1.40]
 * Target Point 5: (38.31, -22.06, 100.00)
 * This tests whether the fixed tryAlpha can find a strict solution.
 */
public class DiagTrajIK2 {
    
    public static void main(String[] args) {
        double px = 38.31, py = -22.06, pz = 100.00;
        boolean isRight = true;
        
        // Previous trajectory solution (warm start)
        double[] qRef = {14.36, -51.21, 87.68, -100.69, -90.00, 1.40};
        double preferredYaw = 44.8;  // From Point 4's YawOffset
        
        System.out.println("=== SIMULATING EXACT TRAJECTORY SOLVER FOR POINT 5 ===");
        System.out.printf("Target: (%.2f, %.2f, %.2f)\n", px, py, pz);
        System.out.printf("qRef (warm start): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
            qRef[0], qRef[1], qRef[2], qRef[3], qRef[4], qRef[5]);
        System.out.printf("preferredYaw: %.1f\n\n", preferredYaw);
        
        // Test with OLD tryAlpha behavior (warm start only, conditional fallback)
        System.out.println("--- OLD BEHAVIOR (warm start only, qHome[0]=qRef[0]) ---");
        testTryAlpha(px, py, pz, isRight, qRef, preferredYaw, false);
        
        System.out.println("\n--- NEW BEHAVIOR (always cold-start, qHome[0]=yaw) ---");
        testTryAlpha(px, py, pz, isRight, qRef, preferredYaw, true);
    }
    
    static void testTryAlpha(double px, double py, double pz, boolean isRight, 
                              double[] qRef, double preferredYaw, boolean useNewBehavior) {
        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
        
        double bestErr = Double.MAX_VALUE;
        double bestAlpha = 0, bestYaw = 0;
        double[] bestQ = null;
        int strictCount = 0;
        
        // Scan alpha range like trajectory solver: ±18° from last alpha=-51
        double lastAlpha = -51.0;
        for (double alpha = lastAlpha - 18; alpha <= lastAlpha + 18; alpha += 1.0) {
            double alpha_rad = Math.toRadians(alpha);
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            
            Double[] yawOffsets = { 0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -75.0, 75.0, -90.0, 90.0 };
            if (!Double.isNaN(preferredYaw)) {
                Arrays.sort(yawOffsets, (a, b) -> Double.compare(Math.abs(a - preferredYaw), Math.abs(b - preferredYaw)));
            }
            
            for (double offsetDeg : yawOffsets) {
                double yaw = q1_base + Math.toRadians(offsetDeg);
                double cy = Math.cos(yaw), sy = Math.sin(yaw);
                double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                double[][] R_target = multiplyMatrices(R_z, R_y);
                
                // Strategy 1: Warm start
                double[] qInit = new double[NUM_JOINTS];
                for (int i = 0; i < NUM_JOINTS; i++) qInit[i] = Math.toRadians(qRef[i]);
                
                double[] q = solveIK(px, py, pz, R_target, qInit, isRight);
                if (q != null && isWithinLimits(q, isRight)) {
                    double err = computePosErr(q, px, py, pz, isRight);
                    if (err < bestErr) { bestErr = err; bestAlpha = alpha; bestYaw = offsetDeg; bestQ = q; }
                    if (err * 10.0 < 1.0) strictCount++;
                }
                
                if (useNewBehavior) {
                    // Strategy 2: Cold start with geometric yaw
                    for (double q2_val : new double[]{1.2, 0.6, 0.0, -0.6, -1.2}) {
                        double[] qHome = new double[NUM_JOINTS];
                        qHome[0] = yaw;
                        qHome[1] = q2_val;
                        qHome[2] = 0.3;
                        qHome[3] = Math.toRadians(-35.0);
                        double[] q2 = solveIK(px, py, pz, R_target, qHome, isRight);
                        if (q2 != null && isWithinLimits(q2, isRight)) {
                            double err = computePosErr(q2, px, py, pz, isRight);
                            if (err < bestErr) { bestErr = err; bestAlpha = alpha; bestYaw = offsetDeg; bestQ = q2; }
                            if (err * 10.0 < 1.0) strictCount++;
                        }
                    }
                    
                    // Strategy 3: Cold start with qRef[0]
                    if (Math.abs(qInit[0] - yaw) > 0.15) {
                        for (double q2_val : new double[]{0.6, 0.0, -0.6}) {
                            double[] qAlt = new double[NUM_JOINTS];
                            qAlt[0] = qInit[0];
                            qAlt[1] = q2_val;
                            qAlt[2] = 0.3;
                            qAlt[3] = Math.toRadians(-35.0);
                            double[] q3 = solveIK(px, py, pz, R_target, qAlt, isRight);
                            if (q3 != null && isWithinLimits(q3, isRight)) {
                                double err = computePosErr(q3, px, py, pz, isRight);
                                if (err < bestErr) { bestErr = err; bestAlpha = alpha; bestYaw = offsetDeg; bestQ = q3; }
                                if (err * 10.0 < 1.0) strictCount++;
                            }
                        }
                    }
                } else {
                    // OLD: Only fallback when warm start fails
                    double err0 = (q != null && isWithinLimits(q, isRight)) ? computePosErr(q, px, py, pz, isRight) : Double.MAX_VALUE;
                    if (err0 >= 0.1) {
                        for (double q2_val : new double[]{1.2, 0.6, 0.0, -0.6, -1.2}) {
                            double[] qHome = new double[NUM_JOINTS];
                            qHome[0] = qInit[0];  // OLD: used qRef[0]
                            qHome[1] = q2_val;
                            qHome[2] = 0.3;
                            qHome[3] = Math.toRadians(-35.0);
                            double[] q2 = solveIK(px, py, pz, R_target, qHome, isRight);
                            if (q2 != null && isWithinLimits(q2, isRight)) {
                                double err = computePosErr(q2, px, py, pz, isRight);
                                if (err < bestErr) { bestErr = err; bestAlpha = alpha; bestYaw = offsetDeg; bestQ = q2; }
                                if (err * 10.0 < 1.0) strictCount++;
                            }
                        }
                    }
                }
            }
        }
        
        System.out.printf("Best posErr: %.4f mm | Alpha: %.1f | YawOffset: %.1f | Strict solutions: %d\n",
            bestErr * 10.0, bestAlpha, bestYaw, strictCount);
        if (bestQ != null) {
            System.out.printf("Best Q: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                bestQ[0], bestQ[1], bestQ[2], bestQ[3], bestQ[4], bestQ[5]);
        }
        System.out.printf("Verdict: %s\n", bestErr * 10.0 < 1.0 ? "STRICT OK ✅" : "STILL FAILING ❌");
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
