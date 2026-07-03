import kinematics.Kinematics;
import kinematics.JniKinematics;
import java.util.Arrays;

public class TestDetailStep32 {
    public static void main(String[] args) {
        if (!JniKinematics.isLoaded()) {
            System.out.println("Error: JNI library not loaded!");
            return;
        }

        // Center derived from the homed configuration
        double cx = -28.6370, cy = 13.8366, cz = 101.0342;
        double R = 5.0;
        
        // Step 32
        double angle = 32 * 2.0 * Math.PI / 100.0;
        double px = cx + R * Math.cos(angle);
        double py = cy + R * Math.sin(angle);
        double pz = cz;

        // Step 31 solutions as qInit
        double[] qJavaInitDeg = { 21.172638, 48.247781, -17.910469, -47.235046, 52.380661, -9.588246 };
        double[] qJniInitDeg = { 21.597973, 48.252630, -17.233676, -47.845397, 52.529053, -9.955039 };

        double[] qJavaInitRad = new double[6];
        double[] qJniInitRad = new double[6];
        for (int i = 0; i < 6; i++) {
            qJavaInitRad[i] = Math.toRadians(qJavaInitDeg[i]);
            qJniInitRad[i] = Math.toRadians(qJniInitDeg[i]);
        }

        double[] initialQ = { 0.0, Math.toRadians(30.0), Math.toRadians(-30.0), Math.toRadians(-45.0), Math.toRadians(30.0), 0.0 };
        double[][] T_init = Kinematics.computeFKMatrix(initialQ, true);
        double[][] R_target = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                R_target[r][c] = T_init[r][c];
            }
        }

        System.out.printf("Target Position: (%.6f, %.6f, %.6f)\n", px, py, pz);

        // Run Java with Java Init
        System.out.println("\n=== Java Solver with Java Init ===");
        Kinematics.solverMode = 0;
        double[] qJava = Kinematics.solveIK(px, py, pz, R_target, qJavaInitRad.clone(), true);
        System.out.println("Java result: " + Arrays.toString(qJava));

        // Run Java with JNI Init
        System.out.println("\n=== Java Solver with JNI Init ===");
        double[] qJavaWithJniInit = Kinematics.solveIK(px, py, pz, R_target, qJniInitRad.clone(), true);
        System.out.println("Java result with JNI Init: " + Arrays.toString(qJavaWithJniInit));

        // Run JNI with JNI Init
        System.out.println("\n=== JNI Solver with JNI Init ===");
        double[] qJni = JniKinematics.solveIKNative(px, py, pz, R_target, qJniInitRad.clone(), true, 1);
        System.out.println("JNI result: " + Arrays.toString(qJni));
    }
}
