import kinematics.Kinematics;
import static kinematics.Kinematics.*;

public class FindAngles {
    public static void main(String[] args) {
        System.out.println("Scanning for coordinates reachable by BOTH arms symmetrically...");
        
        double bestLowX = 0, bestLowY = 0, bestLowZ = 0;
        double bestHighX = 0, bestHighY = 0, bestHighZ = 0;
        double[] qRightLow = null, qLeftLow = null;
        double[] qRightHigh = null, qLeftHigh = null;
        
        // Scan for Grab Point (Low)
        outerLow:
        for (double z = 70.0; z <= 95.0; z += 5.0) {
            for (double y = 20.0; y <= 35.0; y += 5.0) {
                for (double x = 15.0; x <= 35.0; x += 5.0) {
                    double[] qR = findBestIK(x, y, z, true);
                    double[] qL = findBestIK(x, -y, z, false);
                    
                    if (qR != null && qL != null) {
                        bestLowX = x;
                        bestLowY = y;
                        bestLowZ = z;
                        qRightLow = qR;
                        qLeftLow = qL;
                        break outerLow;
                    }
                }
            }
        }
        
        // Scan for Lift Point (High)
        outerHigh:
        for (double z = 100.0; z <= 125.0; z += 5.0) {
            for (double y = 20.0; y <= 35.0; y += 5.0) {
                for (double x = 15.0; x <= 35.0; x += 5.0) {
                    double[] qR = findBestIK(x, y, z, true);
                    double[] qL = findBestIK(x, -y, z, false);
                    
                    if (qR != null && qL != null) {
                        bestHighX = x;
                        bestHighY = y;
                        bestHighZ = z;
                        qRightHigh = qR;
                        qLeftHigh = qL;
                        break outerHigh;
                    }
                }
            }
        }
        
        if (qRightLow != null && qLeftLow != null) {
            System.out.printf("FOUND LOW POINT: (X=%.1f, Y=%.1f, Z=%.1f)\n", bestLowX, bestLowY, bestLowZ);
            System.out.print("  Right joints: { ");
            for (int i=0; i<6; i++) System.out.printf("%.2f%s", Math.toDegrees(qRightLow[i]), i<5?", ":"");
            System.out.println(" }");
            System.out.print("  Left joints:  { ");
            for (int i=0; i<6; i++) System.out.printf("%.2f%s", Math.toDegrees(qLeftLow[i]), i<5?", ":"");
            System.out.println(" }");
        } else {
            System.out.println("NO REACHABLE LOW POINT FOUND");
        }
        
        if (qRightHigh != null && qLeftHigh != null) {
            System.out.printf("FOUND HIGH POINT: (X=%.1f, Y=%.1f, Z=%.1f)\n", bestHighX, bestHighY, bestHighZ);
            System.out.print("  Right joints: { ");
            for (int i=0; i<6; i++) System.out.printf("%.2f%s", Math.toDegrees(qRightHigh[i]), i<5?", ":"");
            System.out.println(" }");
            System.out.print("  Left joints:  { ");
            for (int i=0; i<6; i++) System.out.printf("%.2f%s", Math.toDegrees(qLeftHigh[i]), i<5?", ":"");
            System.out.println(" }");
        } else {
            System.out.println("NO REACHABLE HIGH POINT FOUND");
        }
    }

    private static double[] findBestIK(double px, double py, double pz, boolean isRight) {
        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
        
        // Scan alpha from -90 to 0 (pointing downwards or forwards)
        for (double alphaDeg = -90; alphaDeg <= 0; alphaDeg += 15) {
            double alpha_rad = Math.toRadians(alphaDeg);
            double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
            double[][] R_rot = {{0, -ca, -sa}, {1, 0, 0}, {0, -sa, ca}};
            
            for (double yawOff : new double[]{0, -15, 15, -30, 30}) {
                double yaw = q1_base + Math.toRadians(yawOff);
                double[][] R_target;
                if (isRight) {
                    double cy = Math.cos(yaw), sy = Math.sin(yaw);
                    double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                    R_target = mul3(R_z, R_rot);
                } else {
                    double yawR = -yaw;
                    double cyR = Math.cos(yawR), syR = Math.sin(yawR);
                    double[][] R_z_right = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
                    double[][] R_target_right = mul3(R_z_right, R_rot);
                    R_target = new double[][] {
                        {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
                        { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
                        { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
                    };
                }
                
                for (double q2 : new double[]{-0.5, 0.0, 0.5}) {
                    for (double q3deg : new double[]{60.0, 100.0, 140.0}) {
                        double q3 = Math.toRadians(isRight ? q3deg : -q3deg);
                        for (double q4deg : new double[]{-45.0, -75.0}) {
                            double q4 = Math.toRadians(isRight ? q4deg : -q4deg);
                            double[] ws = {q1_base, q2, q3, q4, 0, 0};
                            double[] sol = solveIK(px, py, pz, R_target, ws, isRight);
                            if (sol != null && isWithinLimits(sol, isRight)) {
                                return sol;
                            }
                        }
                    }
                }
            }
        }
        return null;
    }

    private static double[][] mul3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        return C;
    }
}
