package scratch;

public class TestWristSolve {
    public static void main(String[] args) {
        double px = 38.41, py = 32.65, pz = 80.0;
        double alpha_rad = Math.toRadians(-60);
        double q1_target = Math.atan2(py, px);
        double c1 = Math.cos(q1_target), s1 = Math.sin(q1_target);
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_target = TestFK.multiply4x4(
            new double[][]{ {c1,-s1,0,0},{s1,c1,0,0},{0,0,1,0},{0,0,0,1} },
            new double[][]{ {ca,0,sa,0},{0,1,0,0},{-sa,0,ca,0},{0,0,0,1} }
        );
        
        double[][] R3 = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R3[i][j] = R_target[i][j];
            }
        }
        
        double[] q = {
            Math.toRadians(13.0),
            Math.toRadians(26.0),
            Math.toRadians(44.0),
            Math.toRadians(-38.0),
            0,
            0
        };
        
        double d2 = 20.0;
        double[][] params = {
            { 0, 125, 0, -Math.PI / 2, q[0] },
            { -Math.PI / 2, d2, 0, -Math.PI / 2, q[1] },
            { -Math.PI / 2, 0, 0, -Math.PI, q[2] },
            { 0, 0, 20.0, -Math.PI / 2, q[3] }
        };
        
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        for (int i = 0; i < 4; i++) {
            T = TestFK.multiply4x4(T, TestFK.getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
        }
        
        double[][] R4 = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R4[i][j] = T[i][j];
            }
        }
        
        double[][] R4T = transpose(R4);
        double[][] Rw = multiply3x3(R4T, R3);
        
        System.out.println("R_wrist matrix:");
        print3x3(Rw);
        
        System.out.println("\nSearching for q5 and q6 matching R_wrist in [-180, 180]...");
        double minDiff = Double.MAX_VALUE;
        double bestQ5 = 0, bestQ6 = 0;
        
        for (double q5 = -180; q5 <= 180; q5 += 0.5) {
            for (double q6 = -180; q6 <= 180; q6 += 0.5) {
                double r5_rad = Math.toRadians(q5);
                double r6_rad = Math.toRadians(q6);
                
                double[][] R5 = getRotationOnly(-Math.PI / 2, -Math.PI / 2, r5_rad);
                double[][] R6 = getRotationOnly(-Math.PI / 2, 0, r6_rad);
                double[][] R56 = multiply3x3(R5, R6);
                
                double diff = 0;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        diff += Math.abs(R56[i][j] - Rw[i][j]);
                    }
                }
                if (diff < minDiff) {
                    minDiff = diff;
                    bestQ5 = q5;
                    bestQ6 = q6;
                }
            }
        }
        System.out.printf("Closest match diff: %.4f at q5 = %.1f, q6 = %.1f\n", minDiff, bestQ5, bestQ6);
    }
    
    public static double[][] transpose(double[][] A) {
        double[][] B = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                B[j][i] = A[i][j];
        return B;
    }
    
    public static double[][] multiply3x3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return C;
    }
    
    public static double[][] getRotationOnly(double alpha, double offset, double q) {
        double theta = q + offset;
        double ct = Math.cos(theta), st = Math.sin(theta);
        double ca = Math.cos(alpha), sa = Math.sin(alpha);
        return new double[][] {
            { ct, -st, 0 },
            { st * ca, ct * ca, -sa },
            { st * sa, ct * sa, ca }
        };
    }
    
    public static void print3x3(double[][] A) {
        for (int i = 0; i < 3; i++) {
            System.out.printf("  [%+6.3f, %+6.3f, %+6.3f]\n", A[i][0], A[i][1], A[i][2]);
        }
    }
}
