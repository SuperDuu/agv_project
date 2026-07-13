public class TestGripperCrossbar {
    public static void main(String[] args) {
        double[] lowPickRight = { 0.0, 0.0, 20.0, -35.0, 0.0, 0.0 };
        double[] highPlaceRight = { 0.0, 0.0, 20.0, -35.0, 0.0, 0.0 };

        System.out.println("Low Pick Right:");
        checkCrossbar(lowPickRight);
        
        System.out.println("\nHigh Place Right:");
        checkCrossbar(highPlaceRight);
    }

    private static void checkCrossbar(double[] anglesDeg) {
        double[][] T = computeMDHMatrix(anglesDeg, true);
        
        double ux = T[0][2], uy = T[1][2], uz = T[2][2]; 
        double nx = T[0][0], ny = T[1][0], nz = T[2][0]; 

        double bx = uy * nz - uz * ny;
        double by = uz * nx - ux * nz;
        double bz = ux * ny - uy * nx;
        double blen = Math.sqrt(bx*bx + by*by + bz*bz);
        if (blen > 1e-6) {
            bx /= blen; by /= blen; bz /= blen;
        }

        // Tilt of the crossbar relative to horizontal:
        double tiltRad = Math.asin(bz);
        double tiltDeg = Math.toDegrees(tiltRad);

        System.out.printf("  Tool Z (finger direction): (%.4f, %.4f, %.4f)\n", ux, uy, uz);
        System.out.printf("  Crossbar Vector: (%.4f, %.4f, %.4f)\n", bx, by, bz);
        System.out.printf("  Crossbar Tilt relative to ground: %.2f degrees\n", tiltDeg);
    }

    private static double[][] computeMDHMatrix(double[] anglesDeg, boolean isRight) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double L0 = 130.0;
        double L1 = 0.0;
        double L2 = 32.0;
        double L3 = 0.0;
        double L4 = 20.0;
        double L5 = 25.0;
        double L6 = 0.0;
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(anglesDeg[0]) },
                { -Math.PI / 2, d2, 0, -Math.PI / 2, Math.toRadians(anglesDeg[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(anglesDeg[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(anglesDeg[3]) },
                { -Math.PI / 2, L5 + L6, 0, 0, Math.toRadians(anglesDeg[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(anglesDeg[5]) }
        };
        for (int i = 0; i < 6; i++) {
            T = multiply4x4(T, getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
        }
        T = multiply4x4(T, getToolMatrix());
        return T;
    }

    private static double[][] getMDHMatrix(double alpha, double d, double a, double theta_offset, double theta) {
        double ct = Math.cos(theta + theta_offset);
        double st = Math.sin(theta + theta_offset);
        double ca = Math.cos(alpha);
        double sa = Math.sin(alpha);
        return new double[][] {
                { ct, -st, 0, a },
                { st * ca, ct * ca, -sa, -sa * d },
                { st * sa, ct * sa, ca, ca * d },
                { 0, 0, 0, 1 }
        };
    }

    private static double[][] getToolMatrix() {
        double L7 = 15.0;
        return new double[][] {
                { 0, -1, 0, 0 },
                { 0, 0, -1, -L7 },
                { 1, 0, 0, 0 },
                { 0, 0, 0, 1 }
        };
    }

    private static double[][] multiply4x4(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }
}
