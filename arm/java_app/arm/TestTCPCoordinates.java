public class TestTCPCoordinates {
    public static void main(String[] args) {
        double[][] keyframes = {
            { 0.0, 20.0, 48.0, -17.0, 40.0, 10.0 }, // lowPickRight
            { 0.0, 35.0, 48.0, -17.0, 40.0, 10.0 }, // lowHoverRight
            { 0.0, -64.0, 42.0, -19.0, -35.0, -45.0 }, // highPlaceRight
            { 0.0, -66.0, 42.0, -19.0, -35.0, -45.0 }  // highHoverRight
        };

        String[] names = { "lowPickRight", "lowHoverRight", "highPlaceRight", "highHoverRight" };

        for (int i = 0; i < keyframes.length; i++) {
            System.out.println(names[i] + ":");
            double[][] pts = computeAllJoints3DForAngles(keyframes[i], true);
            double[] tcp = pts[7];
            System.out.printf("  TCP: (%.2f, %.2f, %.2f)\n", tcp[0], tcp[1], tcp[2]);
            
            double[][] T = computeFKMatrix(keyframes[i]);
            double ux = T[0][2], uy = T[1][2], uz = T[2][2]; 
            double nx = T[0][0], ny = T[1][0], nz = T[2][0]; 

            double bx = uy * nz - uz * ny;
            double by = uz * nx - ux * nz;
            double bz = ux * ny - uy * nx;
            double blen = Math.sqrt(bx*bx + by*by + bz*bz);
            if (blen > 1e-6) {
                bx /= blen; by /= blen; bz /= blen;
            }

            double fingerTilt = Math.toDegrees(Math.asin(uz));
            double crossbarTilt = Math.toDegrees(Math.asin(bz));
            System.out.printf("  Finger Tilt: %.2f deg\n", fingerTilt);
            System.out.printf("  Crossbar Tilt: %.2f deg\n", crossbarTilt);
        }
    }

    private static double[][] computeFKMatrix(double[] anglesDeg) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double L0 = 130.0;
        double L1 = 0.0;
        double L2 = 32.0;
        double L3 = 0.0;
        double L4 = 20.0;
        double L5 = 25.0;
        double L6 = 0.0;
        double d2 = 32.0;
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

    private static double[][] computeAllJoints3DForAngles(double[] anglesDeg, boolean isRight) {
        double[][] pts = new double[8][3];
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        pts[0] = new double[] { 0, 0, 0 };
        double L0 = 130.0;
        double L1 = 0.0;
        double L2 = 32.0;
        double L3 = 0.0;
        double L4 = 20.0;
        double L5 = 25.0;
        double L6 = 0.0;
        double L7 = 15.0;
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
            pts[i + 1] = new double[] { T[0][3], T[1][3], T[2][3] };
        }
        T = multiply4x4(T, getToolMatrix());
        pts[7] = new double[] { T[0][3], T[1][3], T[2][3] };
        return pts;
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
