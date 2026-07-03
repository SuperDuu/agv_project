import kinematics.Kinematics;
import java.util.Arrays;

public class PrintHomedFK {
    public static void main(String[] args) {
        double[] q = { 0.0, Math.toRadians(30.0), Math.toRadians(-30.0), Math.toRadians(-45.0), Math.toRadians(30.0), 0.0 };
        double[][] T = Kinematics.computeFKMatrix(q, true);
        System.out.printf("Homed FK Position: (%.4f, %.4f, %.4f)\n", T[0][3], T[1][3], T[2][3]);
    }
}
