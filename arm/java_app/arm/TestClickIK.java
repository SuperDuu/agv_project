import kinematics.Kinematics;
import static kinematics.Kinematics.*;

/**
 * Comprehensive workspace analysis:
 * - Scans all valid joint combinations to map reachable workspace
 * - Reports actual X/Y/Z min-max bounds  
 * - Identifies unreachable zones
 */
public class TestClickIK {
    public static void main(String[] args) {
        Kinematics.solverMode = 0;
        System.out.println("=== Workspace Analysis (Right Arm, FREE Gripper Mode) ===");
        System.out.println("Joint limits:");
        System.out.printf("  q1=[%.0f,%.0f]  q2=[%.0f,%.0f]  q3=[%.0f,%.0f]\n",
            JOINT_MIN_RIGHT[0], JOINT_MAX_RIGHT[0],
            JOINT_MIN_RIGHT[1], JOINT_MAX_RIGHT[1],
            JOINT_MIN_RIGHT[2], JOINT_MAX_RIGHT[2]);
        System.out.printf("  q4=[%.0f,%.0f]  q5=[%.0f,%.0f]  q6=[%.0f,%.0f]\n\n",
            JOINT_MIN_RIGHT[3], JOINT_MAX_RIGHT[3],
            JOINT_MIN_RIGHT[4], JOINT_MAX_RIGHT[4],
            JOINT_MIN_RIGHT[5], JOINT_MAX_RIGHT[5]);

        double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
        double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;
        double zMin = Double.MAX_VALUE, zMax = -Double.MAX_VALUE;
        int count = 0;

        // Coarse FK scan over all valid joint angles
        int step = 15; // degrees step for speed
        for (int q1 = (int)JOINT_MIN_RIGHT[0]; q1 <= (int)JOINT_MAX_RIGHT[0]; q1 += step) {
            for (int q2 = (int)JOINT_MIN_RIGHT[1]; q2 <= (int)JOINT_MAX_RIGHT[1]; q2 += step) {
                for (int q3 = (int)JOINT_MIN_RIGHT[2]; q3 <= (int)JOINT_MAX_RIGHT[2]; q3 += step) {
                    for (int q4 = (int)JOINT_MIN_RIGHT[3]; q4 <= (int)JOINT_MAX_RIGHT[3]; q4 += step) {
                        // q5/q6 don't affect TCP position, skip for speed
                        double[] rad = {
                            Math.toRadians(q1), Math.toRadians(q2),
                            Math.toRadians(q3), Math.toRadians(q4),
                            0, 0
                        };
                        double[][] T = computeFKMatrix(rad, true);
                        double x = T[0][3], y = T[1][3], z = T[2][3];
                        if (xMin > x) xMin = x;
                        if (xMax < x) xMax = x;
                        if (yMin > y) yMin = y;
                        if (yMax < y) yMax = y;
                        if (zMin > z) zMin = z;
                        if (zMax < z) zMax = z;
                        count++;
                    }
                }
            }
        }

        System.out.printf("FK samples: %d\n", count);
        System.out.printf("Workspace bounds (mm):\n");
        System.out.printf("  X: [%.1f, %.1f]  range=%.1f\n", xMin, xMax, xMax-xMin);
        System.out.printf("  Y: [%.1f, %.1f]  range=%.1f\n", yMin, yMax, yMax-yMin);
        System.out.printf("  Z: [%.1f, %.1f]  range=%.1f\n\n", zMin, zMax, zMax-zMin);

        // Test IK for a grid of points inside the workspace
        System.out.println("=== IK Grid Test (FREE mode, alpha scan [-90,30] step 30) ===");
        int reach = 0, total = 0;
        int gridStep = 15;
        int[] testX = range((int)Math.ceil(xMin), (int)Math.floor(xMax), gridStep);
        int[] testY = range((int)Math.ceil(yMin), (int)Math.floor(yMax), gridStep);
        int[] testZ = range((int)Math.ceil(zMin), (int)Math.floor(zMax), gridStep);

        for (int x : testX) {
            for (int y : testY) {
                for (int z : testZ) {
                    total++;
                    if (canReach(x, y, z, true)) reach++;
                }
            }
        }
        System.out.printf("Grid %dx%dx%d (step=%dmm): %d/%d reachable = %.1f%%\n",
            testX.length, testY.length, testZ.length, gridStep,
            reach, total, 100.0*reach/total);

        // Show some representative reachable points
        System.out.println("\nSample reachable points:");
        int shown = 0;
        outer:
        for (int x : testX) {
            for (int y : testY) {
                for (int z : testZ) {
                    if (canReach(x, y, z, true)) {
                        System.out.printf("  (%4d, %4d, %4d)\n", x, y, z);
                        if (++shown >= 20) break outer;
                    }
                }
            }
        }

        // Key workspace sections
        System.out.println("\n=== Workspace at Z levels ===");
        for (int z : new int[]{70, 80, 90, 100, 110, 120, 130}) {
            int r = 0, t = 0;
            for (int x = (int)xMin; x <= (int)xMax; x += 10) {
                for (int y = (int)yMin; y <= (int)yMax; y += 10) {
                    t++;
                    if (canReach(x, y, z, true)) r++;
                }
            }
            System.out.printf("  Z=%3d mm: %d/%d reachable (%.0f%%)\n", z, r, t, 100.0*r/t);
        }
    }

    static boolean canReach(double px, double py, double pz, boolean isRight) {
        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
        // Try multiple alphas (FREE mode: full pitch freedom)
        for (double alphaDeg = -90; alphaDeg <= 30; alphaDeg += 30) {
            double alpha_rad = Math.toRadians(alphaDeg);
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_rot = {{0, -ca, -sa}, {1, 0, 0}, {0, -sa, ca}};
            // Try multiple yaw offsets
            for (double yawOff : new double[]{0, -20, 20, -40, 40}) {
                double yaw = q1_base + Math.toRadians(yawOff);
                double cy = Math.cos(yaw), sy = Math.sin(yaw);
                double[][] Rz = {{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}};
                double[][] Rt = mul3(Rz, R_rot);
                // Multiple warm starts within valid limits
                for (double q2 : new double[]{-1.2, -0.6, 0.0, 0.6, 1.2}) {
                    for (double q3deg : new double[]{60.0, 100.0, 140.0}) {
                        for (double q4deg : new double[]{-35.0, -60.0, -85.0}) {
                            double[] ws = {q1_base, q2, Math.toRadians(q3deg), Math.toRadians(q4deg), 0, 0};
                            double[] sol = solveIK(px, py, pz, Rt, ws, isRight);
                            if (sol != null && isWithinLimits(sol, isRight)) {
                                double[] sr = new double[6];
                                for (int i = 0; i < 6; i++) sr[i] = Math.toRadians(sol[i]);
                                double[][] T = computeFKMatrix(sr, isRight);
                                double err = Math.sqrt(Math.pow(T[0][3]-px,2)+Math.pow(T[1][3]-py,2)+Math.pow(T[2][3]-pz,2));
                                if (err < 1.5) return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    static int[] range(int from, int to, int step) {
        int n = (to - from) / step + 1;
        if (n <= 0) return new int[]{from};
        int[] r = new int[n];
        for (int i = 0; i < n; i++) r[i] = from + i * step;
        return r;
    }

    static double[][] mul3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        return C;
    }
}
