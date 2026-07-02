package scratch;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;

/**
 * Diagnostic tool: For failing trajectory points, sweep ALL alpha/yaw combinations
 * and report what posErr is achievable. This reveals whether the problem is:
 * (A) The point is truly unreachable at Z=100 (FK workspace lie)
 * (B) The IK solver can't converge (Jacobian singularity / bad conditioning)
 * (C) The alpha search window is too narrow (trajectory solver bug)
 */
public class DiagTrajIK {

    // Failing points from user's log (ALL at Z=100)
    static double[][] failingTargets = {
        {38.31, -22.06, 100.00},  // Point 5  posErr=4.6mm
        {38.66, -21.81, 100.00},  // Point 6  posErr=9.0mm
        {41.47, -18.81, 100.00},  // Point 12 posErr=8.2mm
        {42.26, -17.42, 100.00},  // Point 14 posErr=8.0mm
        {43.61, -5.73,  100.00},  // Point 28 posErr=8.7mm
        {46.68, 3.43,   100.00},  // Point 39 posErr=8.8mm
        {47.01, 5.15,   100.00},  // Point 41 posErr=15.0mm
        {47.14, 6.86,   100.00},  // Point 43 posErr=11.8mm
    };

    // Succeeding points nearby for comparison
    static double[][] succeedingTargets = {
        {37.94, -22.30, 100.00},  // Point 4  posErr=0.55mm
        {39.00, -21.53, 100.00},  // Point 7  posErr=0.77mm
        {41.90, -18.13, 100.00},  // Point 13 posErr=0.05mm
        {42.56, -16.68, 100.00},  // Point 15 posErr=0.88mm
    };

    public static void main(String[] args) {
        System.out.println("=== DIAGNOSTIC: TRAJECTORY IK FAILURE ANALYSIS ===\n");
        
        // Part 1: For each failing point, sweep all alphas and find best achievable posErr
        System.out.println("--- PART 1: BEST ACHIEVABLE posErr FOR FAILING POINTS ---");
        System.out.printf("%-35s | %-10s | %-10s | %-12s | %-10s | %-12s%n",
            "Target (X, Y, Z)", "BestErr_mm", "BestAlpha", "BestYawOff", "NumStrict", "CondNum");
        System.out.println("-".repeat(110));
        
        for (double[] target : failingTargets) {
            analyzePoint(target[0], target[1], target[2], true);
        }
        
        System.out.println("\n--- PART 2: SAME ANALYSIS FOR SUCCEEDING POINTS (CONTROL GROUP) ---");
        System.out.printf("%-35s | %-10s | %-10s | %-12s | %-10s | %-12s%n",
            "Target (X, Y, Z)", "BestErr_mm", "BestAlpha", "BestYawOff", "NumStrict", "CondNum");
        System.out.println("-".repeat(110));
        
        for (double[] target : succeedingTargets) {
            analyzePoint(target[0], target[1], target[2], true);
        }
        
        // Part 3: Deep dive on one failing point - convergence analysis
        System.out.println("\n--- PART 3: CONVERGENCE DEEP DIVE ON (38.31, -22.06, 100.00) ---");
        deepDive(38.31, -22.06, 100.00, true);
        
        // Part 4: Check if the FK workspace slice actually covers these points
        System.out.println("\n--- PART 4: FK WORKSPACE SLICE VERIFICATION ---");
        verifyFKSlice(100.0, true);
    }

    static void analyzePoint(double px, double py, double pz, boolean isRight) {
        double bestErr = Double.MAX_VALUE;
        double bestAlpha = 0;
        double bestYawOff = 0;
        int numStrict = 0;  // solutions with posErr < 1mm
        double bestCondNum = Double.MAX_VALUE;
        
        double q1_base = Math.atan2(py, px);
        double[] activeAngles = {0, 0, 10, -30, 0, 0};  // Default home position
        
        for (double alpha = -90; alpha <= 30; alpha += 3.0) {
            double alpha_rad = Math.toRadians(alpha);
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            
            double[] yawOffsets = {0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90};
            
            for (double offsetDeg : yawOffsets) {
                double yaw = q1_base + Math.toRadians(offsetDeg);
                double cy = Math.cos(yaw), sy = Math.sin(yaw);
                double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                double[][] R_target = multiplyMatrices(R_z, R_y);
                
                // Try with default init
                double[] qInit = new double[NUM_JOINTS];
                for (int i = 0; i < NUM_JOINTS; i++) qInit[i] = Math.toRadians(activeAngles[i]);
                
                double[] q = solveIK(px, py, pz, R_target, qInit, isRight);
                if (q != null && isWithinLimits(q, isRight)) {
                    double err = computePosErr(q, px, py, pz, isRight);
                    if (err < bestErr) {
                        bestErr = err;
                        bestAlpha = alpha;
                        bestYawOff = offsetDeg;
                        
                        // Compute Jacobian condition number at this solution
                        double[] qRad = new double[6];
                        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
                        bestCondNum = computeConditionNumber(qRad, isRight);
                    }
                    if (err * 10.0 < 1.0) numStrict++;  // < 1mm
                }
                
                // Also try with multiple q2 initial guesses
                double[] q2_guesses = {1.2, 0.6, 0.0, -0.6, -1.2};
                for (double q2_val : q2_guesses) {
                    double[] qHome = new double[NUM_JOINTS];
                    qHome[0] = q1_base;
                    qHome[1] = q2_val;
                    double[] q2 = solveIK(px, py, pz, R_target, qHome, isRight);
                    if (q2 != null && isWithinLimits(q2, isRight)) {
                        double err = computePosErr(q2, px, py, pz, isRight);
                        if (err < bestErr) {
                            bestErr = err;
                            bestAlpha = alpha;
                            bestYawOff = offsetDeg;
                            double[] qRad = new double[6];
                            for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q2[i]);
                            bestCondNum = computeConditionNumber(qRad, isRight);
                        }
                        if (err * 10.0 < 1.0) numStrict++;
                    }
                }
            }
        }
        
        System.out.printf("(%.2f, %.2f, %.2f) | %8.4f  | %8.1f  | %10.1f  | %8d  | %10.2f%n",
            px, py, pz, bestErr * 10.0, bestAlpha, bestYawOff, numStrict, bestCondNum);
    }
    
    static void deepDive(double px, double py, double pz, boolean isRight) {
        System.out.println("Sweeping ALL alpha values, showing best posErr at each alpha:");
        System.out.printf("%-8s | %-12s | %-12s | %-12s%n", "Alpha", "BestErr_mm", "BestYawOff", "CondNumber");
        System.out.println("-".repeat(60));
        
        double q1_base = Math.atan2(py, px);
        double[] activeAngles = {0, 0, 10, -30, 0, 0};
        
        for (double alpha = -90; alpha <= 30; alpha += 3.0) {
            double alpha_rad = Math.toRadians(alpha);
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            
            double bestErr = Double.MAX_VALUE;
            double bestYaw = 0;
            double bestCond = Double.MAX_VALUE;
            
            double[] yawOffsets = {0, -15, 15, -30, 30, -45, 45, -60, 60, -75, 75, -90, 90};
            for (double offsetDeg : yawOffsets) {
                double yaw = q1_base + Math.toRadians(offsetDeg);
                double cy = Math.cos(yaw), sy = Math.sin(yaw);
                double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                double[][] R_target = multiplyMatrices(R_z, R_y);
                
                double[] qInit = new double[NUM_JOINTS];
                for (int i = 0; i < NUM_JOINTS; i++) qInit[i] = Math.toRadians(activeAngles[i]);
                
                double[] q = solveIK(px, py, pz, R_target, qInit, isRight);
                if (q != null && isWithinLimits(q, isRight)) {
                    double err = computePosErr(q, px, py, pz, isRight);
                    if (err < bestErr) {
                        bestErr = err;
                        bestYaw = offsetDeg;
                        double[] qRad = new double[6];
                        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
                        bestCond = computeConditionNumber(qRad, isRight);
                    }
                }
                
                // Also try fallback init guesses
                double[] q2_guesses = {1.2, 0.6, 0.0, -0.6, -1.2};
                for (double q2_val : q2_guesses) {
                    double[] qHome = new double[NUM_JOINTS];
                    qHome[0] = q1_base;
                    qHome[1] = q2_val;
                    double[] q2 = solveIK(px, py, pz, R_target, qHome, isRight);
                    if (q2 != null && isWithinLimits(q2, isRight)) {
                        double err = computePosErr(q2, px, py, pz, isRight);
                        if (err < bestErr) {
                            bestErr = err;
                            bestYaw = offsetDeg;
                            double[] qRad = new double[6];
                            for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q2[i]);
                            bestCond = computeConditionNumber(qRad, isRight);
                        }
                    }
                }
            }
            
            if (bestErr < Double.MAX_VALUE) {
                String marker = bestErr * 10.0 < 1.0 ? " <<<STRICT" : "";
                System.out.printf("%6.1f  | %10.4f  | %10.1f  | %10.2f%s%n",
                    alpha, bestErr * 10.0, bestYaw, bestCond, marker);
            } else {
                System.out.printf("%6.1f  | %10s  | %10s  | %10s%n", alpha, "NO_SOL", "-", "-");
            }
        }
    }
    
    static void verifyFKSlice(double fixedZ, boolean isRight) {
        // Check if the failing points are ACTUALLY inside the FK-based workspace envelope
        double[] minLim = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        
        System.out.println("Checking if failing targets are inside FK workspace envelope at Z=" + fixedZ);
        
        for (double[] target : failingTargets) {
            double px = target[0], py = target[1];
            double targetR = Math.sqrt(px * px + py * py);
            double targetTheta = Math.toDegrees(Math.atan2(py, px));
            
            // Find closest FK point
            double closestDist = Double.MAX_VALUE;
            double closestZ = 0;
            
            double[] q4_samples = {minLim[3], (minLim[3] + maxLim[3]) / 2.0, maxLim[3]};
            double[] q5_samples = {minLim[4] + 30, (minLim[4] + maxLim[4]) / 2.0, maxLim[4] - 30};
            
            for (double q4 : q4_samples) {
                for (double q5 : q5_samples) {
                    for (double q3 = minLim[2]; q3 <= maxLim[2]; q3 += 2.0) {
                        for (double q2 = minLim[1]; q2 <= maxLim[1]; q2 += 2.0) {
                            double[] q = {0, q2, q3, q4, q5, 0};
                            double[] qRad = new double[6];
                            for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
                            double[][] T = computeFKMatrix(qRad, isRight);
                            double z = T[2][3];
                            
                            if (Math.abs(z - fixedZ) < 0.3) {
                                double r = Math.sqrt(T[0][3] * T[0][3] + T[1][3] * T[1][3]);
                                // Rotate to match target theta
                                for (double q1 = minLim[0]; q1 <= maxLim[0]; q1 += 2.0) {
                                    double rad = Math.toRadians(q1);
                                    double fkX = T[0][3] * Math.cos(rad) - T[1][3] * Math.sin(rad);
                                    double fkY = T[0][3] * Math.sin(rad) + T[1][3] * Math.cos(rad);
                                    double dist = Math.sqrt((fkX - px) * (fkX - px) + (fkY - py) * (fkY - py));
                                    if (dist < closestDist) {
                                        closestDist = dist;
                                        closestZ = z;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            String verdict = closestDist < 1.0 ? "INSIDE" : (closestDist < 3.0 ? "EDGE" : "OUTSIDE");
            System.out.printf("  Target=(%.2f, %.2f) r=%.2f theta=%.1f° | Closest FK dist=%.2f cm, Z=%.2f | %s%n",
                px, py, targetR, targetTheta, closestDist, closestZ, verdict);
        }
    }
    
    static double computePosErr(double[] qDeg, double tx, double ty, double tz, boolean isRight) {
        double[] qRad = new double[6];
        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(qDeg[i]);
        double[][] T = computeFKMatrix(qRad, isRight);
        double dx = T[0][3] - tx;
        double dy = T[1][3] - ty;
        double dz = T[2][3] - tz;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    static double computeConditionNumber(double[] qRad, boolean isRight) {
        double[][] J = computeJacobianEE(qRad, isRight);
        // Frobenius norm as a proxy for condition number
        double norm = 0;
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                norm += J[i][j] * J[i][j];
        double det = Math.abs(compute6x6Determinant(J));
        if (det < 1e-15) return Double.MAX_VALUE;
        return Math.sqrt(norm) / Math.pow(det, 1.0/6.0);
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
