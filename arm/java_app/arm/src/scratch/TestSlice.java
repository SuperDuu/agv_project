package scratch;

import kinematics.Kinematics;

public class TestSlice {
    public static void main(String[] args) {
        System.out.println("Running joint-space slice scan with optimized steps...");
        double fixedZ = 100.0;
        double tolerance = 2.0;
        boolean isRight = true;
        
        double[] minLim = Kinematics.JOINT_MIN_RIGHT;
        double[] maxLim = Kinematics.JOINT_MAX_RIGHT;
        
        double q4_min = minLim[3];
        double q4_max = maxLim[3];
        double[] q4_samples = { q4_min, (q4_min + q4_max) / 2.0, q4_max };

        double q5_min = minLim[4];
        double q5_max = maxLim[4];
        double[] q5_samples = { q5_min + 30.0, (q5_min + q5_max) / 2.0, q5_max - 30.0 };
        
        long startTime = System.currentTimeMillis();
        int count = 0;
        
        for (double q4 : q4_samples) {
            for (double q5 : q5_samples) {
                for (double q3 = minLim[2]; q3 <= maxLim[2]; q3 += 4.0) {
                    for (double q2 = minLim[1]; q2 <= maxLim[1]; q2 += 4.0) {
                        double[] q = { 0, q2, q3, q4, q5, 0 };
                        double[] qRad = new double[6];
                        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
                        double[][] T = Kinematics.computeFKMatrix(qRad, isRight);
                        double z = T[2][3];
                        
                        if (Math.abs(z - fixedZ) < tolerance) {
                            // Rotate around Z axis
                            for (double q1 = minLim[0]; q1 <= maxLim[0]; q1 += 4.0) {
                                double rad = Math.toRadians(q1);
                                double x = T[0][3] * Math.cos(rad) - T[1][3] * Math.sin(rad);
                                double y = T[0][3] * Math.sin(rad) + T[1][3] * Math.cos(rad);
                                count++;
                            }
                        }
                    }
                }
            }
        }
        long endTime = System.currentTimeMillis();
        System.out.println("Total points found in slice: " + count);
        System.out.println("Scan took: " + (endTime - startTime) + " ms");
    }
}
