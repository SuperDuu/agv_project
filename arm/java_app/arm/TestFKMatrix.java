import kinematics.Kinematics;

public class TestFKMatrix {
    public static void main(String[] args) {
        double[] qDeg = { 0.0, -90.0, 137.0, -83.0, 0.0, 30.0 };
        double[] q = new double[6];
        for (int i = 0; i < 6; i++) {
            q[i] = Math.toRadians(qDeg[i]);
        }
        double[][] T = Kinematics.computeFKMatrix(q, true);
        System.out.println("T_end rotation part:");
        for (int i = 0; i < 3; i++) {
            System.out.printf("  [%.4f, %.4f, %.4f] | pos: %.4f\n", T[i][0], T[i][1], T[i][2], T[i][3]);
        }
    }
}
