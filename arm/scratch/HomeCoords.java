import kinematics.Kinematics;

public class HomeCoords {
    public static void main(String[] args) {
        double[] qRight = { 0, 0, 10.0, -30.0, 0, 0 };
        double[] qLeft = { 0, 0, -10.0, 30.0, 0, 0 };

        double[] qRadRight = new double[6];
        double[] qRadLeft = new double[6];
        for (int i = 0; i < 6; i++) {
            qRadRight[i] = Math.toRadians(qRight[i]);
            qRadLeft[i] = Math.toRadians(qLeft[i]);
        }

        double[][] TRight = Kinematics.computeFKMatrix(qRadRight, true);
        double[][] TLeft = Kinematics.computeFKMatrix(qRadLeft, false);

        System.out.printf("Right Arm Home: (X=%.2f, Y=%.2f, Z=%.2f)\n", TRight[0][3], TRight[1][3], TRight[2][3]);
        System.out.printf("Left Arm Home: (X=%.2f, Y=%.2f, Z=%.2f)\n", TLeft[0][3], TLeft[1][3], TLeft[2][3]);
    }
}
