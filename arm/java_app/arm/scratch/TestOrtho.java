import kinematics.Kinematics;

public class TestOrtho {
    public static void main(String[] args) {
        // Let's create a slightly non-orthogonal matrix
        double[][] R = {
            { 0.999, 0.012, 0.045 },
            { -0.012, 0.999, 0.015 },
            { -0.045, -0.015, 0.999 }
        };
        
        System.out.println("Original R:");
        printMatrix(R);
        
        double[][] Q = kinematics.Kinematics.orthonormalize3x3(R);
        System.out.println("Orthonormalized R:");
        printMatrix(Q);
    }
    
    private static void printMatrix(double[][] M) {
        for(int i=0; i<3; i++) {
            System.out.printf("  [%.6f, %.6f, %.6f]\n", M[i][0], M[i][1], M[i][2]);
        }
    }
}
