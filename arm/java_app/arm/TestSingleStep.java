import kinematics.Kinematics;
import kinematics.JniKinematics;
import java.util.Arrays;

public class TestSingleStep {
    public static void main(String[] args) {
        if (!JniKinematics.isLoaded()) {
            System.out.println("Error: JNI library not loaded!");
            return;
        }

        // Center derived from the homed configuration
        double cx = -28.6370, cy = 13.8366, cz = 101.0342;
        double R = 5.0;
        
        // Step 7 angle
        double angle = 7 * 2.0 * Math.PI / 100.0;
        double px = cx + R * Math.cos(angle);
        double py = cy + R * Math.sin(angle);
        double pz = cz;

        // Homed configuration as qInit
        double[] qInitDeg = { 0.0, 30.0, -30.0, -45.0, 30.0, 0.0 };
        double[] qInitRad = new double[6];
        for (int i = 0; i < 6; i++) {
            qInitRad[i] = Math.toRadians(qInitDeg[i]);
        }

        double[] initialQ = { 0.0, Math.toRadians(30.0), Math.toRadians(-30.0), Math.toRadians(-45.0), Math.toRadians(30.0), 0.0 };
        double[][] T_init = Kinematics.computeFKMatrix(initialQ, true);
        double[][] R_target = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                R_target[r][c] = T_init[r][c];
            }
        }

        System.out.printf("Target Position: (%.15f, %.15f, %.15f)\n", px, py, pz);

        // Run Java
        Kinematics.solverMode = 0;
        double[] qJava = Kinematics.solveIK(px, py, pz, R_target, qInitRad.clone(), true);
        System.out.println("\nJava result:");
        for (int i = 0; i < 6; i++) {
            System.out.printf("  q[%d] = %.15f\n", i, qJava[i]);
        }

        // Run JNI
        Kinematics.solverMode = 1;
        double[] qJni = JniKinematics.solveIKNative(px, py, pz, R_target, qInitRad.clone(), true, 1);
        System.out.println("\nJNI result:");
        for (int i = 0; i < 6; i++) {
            System.out.printf("  q[%d] = %.15f\n", i, qJni[i]);
        }
    }
}
