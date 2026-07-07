import kinematics.Kinematics;
import kinematics.JniKinematics;
import java.util.Arrays;

public class TestSingleStep {
    public static void main(String[] args) {
        if (!JniKinematics.isLoaded()) {
            System.out.println("Error: JNI library not loaded!");
            return;
        }

        // Homed configuration as qInit (Right Arm)
        double[] qInitDeg = { 0.0, 0.0, 20.0, -35.0, 0.0, 0.0 };
        double[] qInitRad = new double[6];
        for (int i = 0; i < 6; i++) {
            qInitRad[i] = Math.toRadians(qInitDeg[i]);
        }

        double[] initialQ = { 0.0, 0.0, Math.toRadians(20.0), Math.toRadians(-35.0), 0.0, 0.0 };
        double[][] T_init = Kinematics.computeFKMatrix(initialQ, true);
        System.out.println("T_init Matrix:");
        for (int r = 0; r < 4; r++) {
            System.out.println(java.util.Arrays.toString(T_init[r]));
        }

        // Center derived from the homed configuration
        double cx = T_init[0][3], cy = T_init[1][3], cz = T_init[2][3];
        double R = 0.5;
        
        // Step 7 angle
        double angle = 7 * 2.0 * Math.PI / 100.0;
        double px = cx + R * Math.cos(angle);
        double py = cy + R * Math.sin(angle);
        double pz = cz;

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
        System.out.println("\nJava result: " + (qJava == null ? "NULL" : java.util.Arrays.toString(qJava)));

        // Run JNI
        Kinematics.solverMode = 1;
        double[] qJni = JniKinematics.solveIKNative(px, py, pz, R_target, qInitRad.clone(), true, 1);
        System.out.println("\nJNI result: " + (qJni == null ? "NULL" : java.util.Arrays.toString(qJni)));
    }
}
