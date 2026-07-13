package gui;

import kinematics.Kinematics;

public class TestNewIK {
    public static void main(String[] args) {
        // Load the JNI library
        try {
            System.loadLibrary("kinematics_jni");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Could not load JNI library: " + e.getMessage());
            return;
        }

        double sharedQ1 = 0.0;
        
        // Target 1: Low Pick (Low Chair Seat = 75 cm, Object top = 86.2 cm)
        // lowChairCenter: X = 39.88, Y = 25.63
        double targetLowX = 39.88;
        double targetLowY = 25.63;
        
        // Target 2: High Place (High Chair Seat = 95 cm, Object top = 106.2 cm)
        // highChairCenter: X = 71.97, Y = -13.77
        double targetHighX = 71.97;
        double targetHighY = -13.77;

        double yaw = 0.0;
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
        double ca = Math.cos(Math.PI), sa = Math.sin(Math.PI);
        double[][] R_rot = { { 0, -ca, -sa }, { 1, 0, 0 }, { 0, -sa, ca } };
        double[][] R_target = multiplyMatrices(R_z, R_rot);

        System.out.println("Low pick solutions at different Z heights:");
        for (double z = 70.0; z <= 110.0; z += 5.0) {
            double[] qInitRad = { 0.0, Math.toRadians(30.0), Math.toRadians(86.0), Math.toRadians(-95.0), Math.toRadians(57.0), Math.toRadians(60.0) };
            double[] sol = Kinematics.solveIK(targetLowX, targetLowY, z, R_target, qInitRad, true);
            if (sol != null) {
                System.out.printf("  Z=%.1f -> q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
                        z, Math.toDegrees(sol[0]), Math.toDegrees(sol[1]), Math.toDegrees(sol[2]),
                        Math.toDegrees(sol[3]), Math.toDegrees(sol[4]), Math.toDegrees(sol[5]));
            } else {
                System.out.printf("  Z=%.1f -> No Solution\n", z);
            }
        }

        System.out.println("\nHigh place solutions at different Z heights:");
        for (double z = 90.0; z <= 135.0; z += 5.0) {
            double[] qInitRad = { 0.0, Math.toRadians(-90.0), Math.toRadians(137.0), Math.toRadians(-83.0), Math.toRadians(0.0), Math.toRadians(30.0) };
            double[] sol = Kinematics.solveIK(targetHighX, targetHighY, z, R_target, qInitRad, true);
            if (sol != null) {
                System.out.printf("  Z=%.1f -> q=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]\n",
                        z, Math.toDegrees(sol[0]), Math.toDegrees(sol[1]), Math.toDegrees(sol[2]),
                        Math.toDegrees(sol[3]), Math.toDegrees(sol[4]), Math.toDegrees(sol[5]));
            } else {
                System.out.printf("  Z=%.1f -> No Solution\n", z);
            }
        }
    }

    private static double[][] multiplyMatrices(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }
}
