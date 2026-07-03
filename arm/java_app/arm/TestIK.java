import kinematics.Kinematics;
import kinematics.JniKinematics;
import java.util.Arrays;

public class TestIK {
    public static void main(String[] args) {
        if (!JniKinematics.isLoaded()) {
            System.out.println("Error: JNI library not loaded!");
            return;
        }

        // 1. Initial angles in degrees: { 0, 0, 10, -30, 0, 0 }
        double[] qDeg = { 0.0, 0.0, 10.0, -30.0, 0.0, 0.0 };
        double[] qRad = new double[6];
        for (int i = 0; i < 6; i++) {
            qRad[i] = Math.toRadians(qDeg[i]);
        }
        boolean isRight = true;

        // Compute FK to get exact target position & rotation matrix
        double[][] T = Kinematics.computeFKMatrix(qRad, isRight);
        double px = T[0][3];
        double py = T[1][3];
        double pz = T[2][3];

        double[][] R_target = {
            { T[0][0], T[0][1], T[0][2] },
            { T[1][0], T[1][1], T[1][2] },
            { T[2][0], T[2][1], T[2][2] }
        };

        System.out.printf("FK Position: (%.4f, %.4f, %.4f)\n", px, py, pz);
        System.out.println("R_target:");
        for (int i = 0; i < 3; i++) {
            System.out.printf("  [%.4f, %.4f, %.4f]\n", R_target[i][0], R_target[i][1], R_target[i][2]);
        }

        // Slightly perturb initial guess to test solver convergence
        double[] qInit = new double[6];
        for (int i = 0; i < 6; i++) {
            qInit[i] = qRad[i] + Math.toRadians(2.0); // perturb by 2 degrees
        }

        // Test all three modes
        System.out.println("\nTesting solvers with perturbed initial guess...");

        // Java
        Kinematics.solverMode = 0;
        double[] qJava = Kinematics.solveIK(px, py, pz, R_target, qInit, isRight);

        // JNI Numerical
        Kinematics.solverMode = 1;
        double[] qJni = Kinematics.solveIK(px, py, pz, R_target, qInit, isRight);

        // JNI IKFast
        Kinematics.solverMode = 2;
        double[] qFast = Kinematics.solveIK(px, py, pz, R_target, qInit, isRight);

        printSol("Java ", qJava);
        printSol("JNI  ", qJni);
        printSol("IKF  ", qFast);
    }

    private static void printSol(String label, double[] q) {
        if (q == null) {
            System.out.println(label + ": FAILED");
        } else {
            System.out.printf("%s: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
                label, q[0], q[1], q[2], q[3], q[4], q[5]);
        }
    }
}
