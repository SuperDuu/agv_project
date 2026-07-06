import kinematics.Kinematics;
import kinematics.JniKinematics;
import java.util.ArrayList;
import java.util.List;

public class TestTrajectoryIK {
    public static void main(String[] args) {
        if (!JniKinematics.isLoaded()) {
            System.out.println("Error: JNI library not loaded!");
            return;
        }

        // Right Arm home configuration: J3=20, J4=-35 (gives q4=J4+J3=-15 = physical home)
        double[] homeRad = { 0.0, 0.0, Math.toRadians(20.0), Math.toRadians(-35.0), 0.0, 0.0 };
        double[][] T_init = Kinematics.computeFKMatrix(homeRad, true);

        // Center the circle at the home EE position
        double cx = T_init[0][3], cy = T_init[1][3], cz = T_init[2][3];
        double R = 2.0;   // 2mm radius — stays well within workspace
        int numPoints = 100;

        System.out.printf("Home FK EE: (%.3f, %.3f, %.3f)\n", cx, cy, cz);

        List<double[]> path = new ArrayList<>();
        for (int i = 0; i <= numPoints; i++) {
            double angle = i * 2.0 * Math.PI / numPoints;
            path.add(new double[]{ cx + R * Math.cos(angle), cy + R * Math.sin(angle), cz });
        }

        // Target orientation: same as home
        double[][] R_target = new double[3][3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                R_target[r][c] = T_init[r][c];

        System.out.println("Comparing Java and JNI step by step...");

        double[] qJava = homeRad.clone();
        double[] qJni  = homeRad.clone();

        int bothFail = 0, javaSoleFail = 0, jniSoleFail = 0, mismatchCount = 0;
        double maxDelta = 0.0;

        for (int i = 0; i < path.size(); i++) {
            double[] pt = path.get(i);

            // Java
            Kinematics.solverMode = 0;
            double[] qSolJava = Kinematics.solveIK(pt[0], pt[1], pt[2], R_target, qJava.clone(), true);

            // JNI
            Kinematics.solverMode = 1;
            double[] qSolJni  = Kinematics.solveIK(pt[0], pt[1], pt[2], R_target, qJni.clone(), true);

            if (qSolJava == null && qSolJni == null) { bothFail++; continue; }

            if (qSolJava == null || qSolJni == null) {
                System.out.printf("Step %3d: Java=%s, JNI=%s\n", i,
                    qSolJava == null ? "FAIL" : "OK",
                    qSolJni  == null ? "FAIL" : "OK");
                if (qSolJava == null) javaSoleFail++; else jniSoleFail++;
                if (qSolJava != null) for (int j = 0; j < 6; j++) qJava[j] = Math.toRadians(qSolJava[j]);
                if (qSolJni  != null) for (int j = 0; j < 6; j++) qJni[j]  = Math.toRadians(qSolJni[j]);
                continue;
            }

            // Both succeeded — verify limits
            boolean javaValid = Kinematics.isWithinLimits(qSolJava, true);
            boolean jniValid  = Kinematics.isWithinLimits(qSolJni,  true);
            if (!javaValid) System.out.printf("Step %3d: Java VIOLATES limits: %s\n", i, java.util.Arrays.toString(qSolJava));
            if (!jniValid)  System.out.printf("Step %3d: JNI  VIOLATES limits: %s\n", i, java.util.Arrays.toString(qSolJni));

            // Compare joint angles
            double stepMax = 0;
            for (int j = 0; j < 6; j++) stepMax = Math.max(stepMax, Math.abs(qSolJava[j] - qSolJni[j]));
            maxDelta = Math.max(maxDelta, stepMax);
            if (stepMax > 0.01) {   // tolerance: 0.01 degree
                mismatchCount++;
                System.out.printf("Step %3d max_delta=%.4f  Java:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]  JNI:[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\n",
                    i, stepMax,
                    qSolJava[0], qSolJava[1], qSolJava[2], qSolJava[3], qSolJava[4], qSolJava[5],
                    qSolJni[0],  qSolJni[1],  qSolJni[2],  qSolJni[3],  qSolJni[4],  qSolJni[5]);
            }

            for (int j = 0; j < 6; j++) {
                qJava[j] = Math.toRadians(qSolJava[j]);
                qJni[j]  = Math.toRadians(qSolJni[j]);
            }
        }

        System.out.printf("\n=== SUMMARY ===\n");
        System.out.printf("  Total steps : %d\n", path.size());
        System.out.printf("  Both failed : %d\n", bothFail);
        System.out.printf("  Java only failed : %d\n", javaSoleFail);
        System.out.printf("  JNI  only failed : %d\n", jniSoleFail);
        System.out.printf("  Mismatch (>0.01°): %d\n", mismatchCount);
        System.out.printf("  Max joint delta  : %.6f°\n", maxDelta);
    }
}
