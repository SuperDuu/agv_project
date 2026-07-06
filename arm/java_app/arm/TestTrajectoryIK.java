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

        // Center derived from the homed configuration
        double cx = -28.6370, cy = 13.8366, cz = 101.0342;
        double R = 5.0;
        int numPoints = 100;
        List<double[]> path = new ArrayList<>();
        for (int i = 0; i <= numPoints; i++) {
            double angle = i * 2.0 * Math.PI / numPoints;
            double px = cx + R * Math.cos(angle);
            double py = cy + R * Math.sin(angle);
            double pz = cz;
            path.add(new double[]{px, py, pz});
        }

        System.out.println("Comparing Java and JNI step by step...");

        double[] qJava = { 0.0, Math.toRadians(30.0), Math.toRadians(15.0), Math.toRadians(5.0), Math.toRadians(30.0), 0.0 };
        double[] qJni = { 0.0, Math.toRadians(30.0), Math.toRadians(15.0), Math.toRadians(5.0), Math.toRadians(30.0), 0.0 };

        double[][] R_target = new double[3][3];
        double[] initialQ = { 0.0, Math.toRadians(30.0), Math.toRadians(15.0), Math.toRadians(5.0), Math.toRadians(30.0), 0.0 };
        double[][] T_init = Kinematics.computeFKMatrix(initialQ, true);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                R_target[r][c] = T_init[r][c];
            }
        }

        for (int i = 0; i < path.size(); i++) {
            double[] pt = path.get(i);

            // Java
            Kinematics.solverMode = 0;
            double[] qInitJava = qJava.clone();
            double[] qSolJava = Kinematics.solveIK(pt[0], pt[1], pt[2], R_target, qInitJava, true);

            // JNI
            Kinematics.solverMode = 1;
            double[] qInitJni = qJni.clone();
            double[] qSolJni = Kinematics.solveIK(pt[0], pt[1], pt[2], R_target, qInitJni, true);

            if (qSolJava == null && qSolJni == null) {
                // Both failed, skip
                continue;
            }

            if (qSolJava == null || qSolJni == null) {
                System.out.printf("Step %d mismatch: Java is %s, JNI is %s\n",
                    i, qSolJava == null ? "FAILED" : "SUCCESS", qSolJni == null ? "FAILED" : "SUCCESS");
                if (qSolJni != null) {
                    System.out.printf("  JNI solution: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
                        qSolJni[0], qSolJni[1], qSolJni[2], qSolJni[3], qSolJni[4], qSolJni[5]);
                    System.out.println("  JNI within limits: " + Kinematics.isWithinLimits(qSolJni, true));
                }
                
                // Keep tracking if one succeeded
                if (qSolJava != null) {
                    for (int j = 0; j < 6; j++) qJava[j] = Math.toRadians(qSolJava[j]);
                }
                if (qSolJni != null) {
                    for (int j = 0; j < 6; j++) qJni[j] = Math.toRadians(qSolJni[j]);
                }
                continue;
            }

            // Both succeeded, compare angles
            boolean mismatch = false;
            for (int j = 0; j < 6; j++) {
                if (Math.abs(qSolJava[j] - qSolJni[j]) > 1e-4) {
                    mismatch = true;
                    break;
                }
            }

            if (mismatch) {
                System.out.printf("Step %d mismatch:\n  Java: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n  JNI : [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",
                    i, qSolJava[0], qSolJava[1], qSolJava[2], qSolJava[3], qSolJava[4], qSolJava[5],
                    qSolJni[0], qSolJni[1], qSolJni[2], qSolJni[3], qSolJni[4], qSolJni[5]);
            }

            // Update state
            for (int j = 0; j < 6; j++) {
                qJava[j] = Math.toRadians(qSolJava[j]);
                qJni[j] = Math.toRadians(qSolJni[j]);
            }
        }
    }
}
