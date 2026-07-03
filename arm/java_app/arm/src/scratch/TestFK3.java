package scratch;

import kinematics.Kinematics;

public class TestFK3 {
    public static void main(String[] args) {
        double[] q = {-29.7, 64.77, 61.48, -66.98, -2.0, 82.98};
        double[] qRad = new double[6];
        for (int i = 0; i < 6; i++) {
            qRad[i] = Math.toRadians(q[i]);
        }
        double[][] T = Kinematics.computeFKMatrix(qRad, true);
        System.out.printf("FK Position: X=%.4f, Y=%.4f, Z=%.4f\n", T[0][3], T[1][3], T[2][3]);
    }
}
