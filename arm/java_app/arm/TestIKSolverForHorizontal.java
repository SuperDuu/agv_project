import kinematics.Kinematics;
import kinematics.RobotTransmission;

public class TestIKSolverForHorizontal {
    public static void main(String[] args) {
        // Target rotation matrix for horizontal finger (pointing +X) and horizontal crossbar (along Y)
        double[][] R_target = {
            { 0, 0, 1 },
            { 0, 1, 0 },
            { -1, 0, 0 }
        };

        System.out.println("Scanning for horizontal configurations (Right arm, Q1 = 0):");
        double[] qInitRad = { 0, 0, Math.toRadians(60), Math.toRadians(-35), 0, 0 };

        for (double z = 110.0; z <= 140.0; z += 1.0) {
            for (double x = 35.0; x <= 65.0; x += 1.0) {
                double[] qRad = Kinematics.solveIK(x, 0.0, z, R_target, qInitRad, true);
                if (qRad != null) {
                    double[] qDeg = new double[6];
                    for (int i = 0; i < 6; i++) {
                        qDeg[i] = Math.toDegrees(qRad[i]);
                    }
                    if (Kinematics.isWithinLimits(qDeg, true)) {
                        // Check parallelogram constraint: 5 <= |q3 - q4| <= 90 in actuator space?
                        // Wait, let's print the configuration and verify the constraints
                        double[] qAct = RobotTransmission.jointToActuator(qDeg[2], qDeg[3], true);
                        double diff = qAct[0] - qAct[1];
                        if (Math.abs(diff) >= 5.0 && Math.abs(diff) <= 90.0) {
                            System.out.printf("  Success: X=%.1f, Z=%.1f -> q=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] | Actuator q3-q4 = %.2f\n",
                                    x, z, qDeg[0], qDeg[1], qDeg[2], qDeg[3], qDeg[4], qDeg[5], diff);
                        }
                    }
                }
            }
        }
    }
}
