import kinematics.Kinematics;

public class DebugStartFK {
    public static void main(String[] args) {
        double[] q = {
            Math.toRadians(0.0),
            Math.toRadians(0.0),
            Math.toRadians(10.0),
            Math.toRadians(-30.0),
            Math.toRadians(0.0),
            Math.toRadians(0.0)
        };
        double[][] T = kinematics.Kinematics.computeFKMatrix(q, true);
        System.out.printf("FK position: X=%.4f, Y=%.4f, Z=%.4f\n", T[0][3], T[1][3], T[2][3]);
        
        // Print rotation matrix
        System.out.println("Rotation matrix:");
        for(int i=0; i<3; i++) {
            System.out.printf("  [%.4f, %.4f, %.4f]\n", T[i][0], T[i][1], T[i][2]);
        }
    }
}
