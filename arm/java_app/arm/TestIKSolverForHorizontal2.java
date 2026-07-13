import kinematics.Kinematics;
import kinematics.RobotTransmission;

public class TestIKSolverForHorizontal2 {
    public static void main(String[] args) {
        System.out.println("Scanning for horizontal configurations (Right arm, Q1 = 0):");
        double[] qInitRad = { 0, 0, Math.toRadians(60), Math.toRadians(-35), 0, 0 };

        for (double z = 100.0; z <= 140.0; z += 1.0) {
            for (double x = 35.0; x <= 65.0; x += 1.0) {
                // Determine target yaw. If the arm points straight along X axis, yaw should be around -90 deg
                // in the shoulder frame. Let's try different yaw angles to find the best match.
                for (double yawDeg = -180.0; yawDeg <= 180.0; yawDeg += 5.0) {
                    double yawRad = Math.toRadians(yawDeg);
                    double cy = Math.cos(yawRad), sy = Math.sin(yawRad);
                    
                    // R_target where Column 0 is (0, 0, -1), Column 1 is (cy, sy, 0), Column 2 is (sy, -cy, 0)
                    double[][] R_target = {
                        { 0, cy, sy },
                        { 0, sy, -cy },
                        {-1,  0,   0 }
                    };

                    double[] qRad = Kinematics.solveIK(x, 0.0, z, R_target, qInitRad, true);
                    if (qRad != null) {
                        double[] qDeg = new double[6];
                        for (int i = 0; i < 6; i++) {
                            qDeg[i] = Math.toDegrees(qRad[i]);
                        }
                        if (Kinematics.isWithinLimits(qDeg, true)) {
                            double[] qAct = RobotTransmission.jointToActuator(qDeg[2], qDeg[3], true);
                            double diff = qAct[0] - qAct[1];
                            if (Math.abs(diff) >= 5.0 && Math.abs(diff) <= 90.0) {
                                System.out.printf("  Success: Z=%.1f, X=%.1f, yaw=%.1f -> q=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] | Actuator diff=%.2f\n",
                                        z, x, yawDeg, qDeg[0], qDeg[1], qDeg[2], qDeg[3], qDeg[4], qDeg[5], diff);
                                break; // Found for this X, Z
                            }
                        }
                    }
                }
            }
        }
    }
}
