package scratch;

public class TestJacobian {
    public static void main(String[] args) {
        double[] q = { 0, 0.5, 0.5, -0.5, 0.5, 0.5 };
        double[][] J = kinematics.Kinematics.computeJacobianEE(q, true);
        System.out.println("Jacobian columns:");
        for (int i = 0; i < 6; i++) {
            System.out.printf("Joint %d: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
                i + 1, J[0][i], J[1][i], J[2][i], J[3][i], J[4][i], J[5][i]);
        }
    }
}
