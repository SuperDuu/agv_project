package app;

import kinematics.Kinematics;
import static kinematics.Kinematics.*;

/**
 * Scans the actual reachable workspace and tests gripper orientation.
 */
public class DebugIK {
    public static void main(String[] args) {
        System.out.println("=== WORKSPACE SCAN & ORIENTATION DEBUG ===\n");
        
        boolean isRight = true;
        double[] homeRad = new double[6];
        double[] homeAngles = { 0, 0, 10, -30, 0, 0 };
        for (int i = 0; i < 6; i++) homeRad[i] = Math.toRadians(homeAngles[i]);
        
        // 1. Scan to find reachable positions
        System.out.println("--- STEP 1: Find reachable positions by FK sampling ---");
        System.out.println("Sampling various joint angles, printing FK positions:\n");
        
        double[][] sampleAngles = {
            {0, 0, 10, -30, 0, 0},      // home
            {0, 0, 30, -60, 0, 0},
            {0, 0, 50, -80, 0, 0},
            {0, 0, -30, 30, 0, 0},
            {0, 30, 10, -30, 0, 0},
            {0, -30, 10, -30, 0, 0},
            {20, 0, 10, -30, 0, 0},
            {-20, 0, 10, -30, 0, 0},
            {0, 0, 0, 0, 0, 0},
            {0, 0, 60, -90, 0, 0},
            {0, 0, 80, -110, 0, 0},
            {0, 45, 30, -60, 0, 0},
            {0, -45, 30, -60, 0, 0},
            {0, 0, 10, -30, 45, 0},
            {0, 0, 10, -30, -45, 0},
        };
        
        for (double[] angles : sampleAngles) {
            double[] rad = new double[6];
            for (int i = 0; i < 6; i++) rad[i] = Math.toRadians(angles[i]);
            double[][] T = computeFKMatrix(rad, isRight);
            double[] uz = { T[0][2], T[1][2], T[2][2] };
            double tilt = Math.toDegrees(Math.acos(Math.min(1.0, Math.abs(uz[2]))));
            System.out.printf("  q=[%4.0f,%4.0f,%4.0f,%4.0f,%4.0f,%4.0f] -> Pos=(%.1f, %.1f, %.1f) tilt=%.1f deg\n",
                angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
                T[0][3], T[1][3], T[2][3], tilt);
        }
        
        // 2. Test IK with soft-constraint at the FK-derived reachable positions  
        System.out.println("\n--- STEP 2: IK with soft-constraint fixedGround at reachable positions ---\n");
        
        for (double[] angles : sampleAngles) {
            double[] rad = new double[6];
            for (int i = 0; i < 6; i++) rad[i] = Math.toRadians(angles[i]);
            double[][] T = computeFKMatrix(rad, isRight);
            double px = T[0][3], py = T[1][3], pz = T[2][3];
            
            // Test soft constraint approach
            testSoftConstraint(px, py, pz, isRight, homeRad);
        }
        
        // 3. Test slider-range positions  
        System.out.println("\n--- STEP 3: IK at slider positions (X=-50..50, Y=-50..50, Z=-20..80) ---\n");
        int reachable = 0, total = 0;
        for (int x = -50; x <= 50; x += 10) {
            for (int y = -50; y <= 50; y += 10) {
                for (int z = 0; z <= 80; z += 20) {
                    total++;
                    double[] q = solveWithAnyAlpha(x, y, z, isRight, homeRad);
                    if (q != null) {
                        reachable++;
                    }
                }
            }
        }
        System.out.printf("Slider grid: %d / %d positions are reachable (%.1f%%)\n", reachable, total, 100.0*reachable/total);
        
        // Show a few that work
        System.out.println("\nReachable slider positions with soft-constraint orientation:");
        for (int x = -10; x <= 20; x += 5) {
            for (int y = -10; y <= 10; y += 5) {
                for (int z = 40; z <= 80; z += 10) {
                    testSoftConstraint(x, y, z, isRight, homeRad);
                }
            }
        }
    }
    
    static void testSoftConstraint(double px, double py, double pz, boolean isRight, double[] qInitRad) {
        double bestCost = Double.MAX_VALUE;
        double[] bestQ = null;
        double bestAlpha = 0;
        
        for (double a = -90; a <= 30; a += 3.0) {
            double[] q = solveWithAlpha(px, py, pz, a, isRight, qInitRad);
            if (q != null) {
                double[] qRad = new double[6];
                for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
                double[][] T = computeFKMatrix(qRad, isRight);
                double posErr = Math.sqrt(Math.pow(T[0][3]-px,2)+Math.pow(T[1][3]-py,2)+Math.pow(T[2][3]-pz,2));
                if (posErr > 1.5) continue;
                double cost = posErr * 200.0 + a * a * 50.0;
                if (cost < bestCost) { bestCost = cost; bestQ = q; bestAlpha = a; }
            }
        }
        
        if (bestQ != null) {
            double[] qRad = new double[6];
            for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(bestQ[i]);
            double[][] T = computeFKMatrix(qRad, isRight);
            double posErr = Math.sqrt(Math.pow(T[0][3]-px,2)+Math.pow(T[1][3]-py,2)+Math.pow(T[2][3]-pz,2));
            double[] uz = { T[0][2], T[1][2], T[2][2] };
            double tilt = Math.toDegrees(Math.acos(Math.min(1.0, Math.abs(uz[2]))));
            System.out.printf("  (%6.1f,%6.1f,%6.1f) -> alpha=%5.1f  tilt=%5.1f deg  posErr=%.3f\n",
                px, py, pz, bestAlpha, tilt, posErr);
        } else {
            System.out.printf("  (%6.1f,%6.1f,%6.1f) -> UNREACHABLE\n", px, py, pz);
        }
    }
    
    static double[] solveWithAnyAlpha(double px, double py, double pz, boolean isRight, double[] qInitRad) {
        for (double a = -90; a <= 30; a += 10.0) {
            double[] q = solveWithAlpha(px, py, pz, a, isRight, qInitRad);
            if (q != null) return q;
        }
        return null;
    }
    
    static double[] solveWithAlpha(double px, double py, double pz, double alphaDeg, boolean isRight, double[] qInitRad) {
        double alpha_rad = Math.toRadians(alphaDeg);
        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        
        double[] yawOffsets = { 0.0, -15.0, 15.0 };
        double[] bestQ = null;
        double bestErr = Double.MAX_VALUE;
        
        for (double offsetDeg : yawOffsets) {
            double yaw = q1_base + Math.toRadians(offsetDeg);
            double cy = Math.cos(yaw), syA = Math.sin(yaw);
            double[][] R_z = { { cy, -syA, 0 }, { syA, cy, 0 }, { 0, 0, 1 } };
            double[][] R_target = mul3x3(R_z, R_y);
            
            double[] q = solveIK(px, py, pz, R_target, qInitRad, isRight);
            if (q != null && withinLimits(q, isRight)) {
                double[] qRad = new double[6];
                for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
                double[][] T = computeFKMatrix(qRad, isRight);
                double err = Math.sqrt(Math.pow(T[0][3]-px,2)+Math.pow(T[1][3]-py,2)+Math.pow(T[2][3]-pz,2));
                if (err < bestErr) { bestErr = err; bestQ = q; }
            }
        }
        return (bestQ != null && bestErr < 1.5) ? bestQ : null;
    }
    
    static boolean withinLimits(double[] q, boolean isRight) {
        double[] mins = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxs = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        for (int i = 0; i < q.length; i++) {
            if (q[i] < mins[i] - 0.5 || q[i] > maxs[i] + 0.5) return false;
        }
        return true;
    }
    
    static double[][] mul3x3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        return C;
    }
}
