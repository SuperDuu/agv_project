import kinematics.RobotTransmission;

public class TestLowChairReach {
    public static void main(String[] args) {
        System.out.println("Scanning workspace for Low Chair Q5=±90 at X ∈ [70, 75], Y ∈ [0, 15], Z ∈ [85, 95]...");
        
        int count = 0;
        for (double q2 = -90.0; q2 <= 90.0; q2 += 1.0) {
            for (double q3 = 20.0; q3 <= 165.0; q3 += 1.0) {
                for (double q4 = -95.0; q4 <= 20.0; q4 += 1.0) {
                    double[] qAct = RobotTransmission.jointToActuator(q3, q4, true);
                    double diff = qAct[0] - qAct[1];
                    if (Math.abs(diff) < 5.0 || Math.abs(diff) > 90.0) {
                        continue;
                    }

                    for (double q5 : new double[] { -90.0, 90.0 }) {
                        for (double q6 = -60.0; q6 <= 60.0; q6 += 1.0) {
                            double[] q = { 0.0, q2, q3, q4, q5, q6 };
                            double[][] T = computeMDHMatrix(q, true);
                            
                            double x = T[0][3];
                            double y = T[1][3];
                            double z = T[2][3];
                            
                            if (x < 70.0 || x > 75.0 || y < 0.0 || y > 15.0 || z < 85.0 || z > 95.0) {
                                continue;
                            }

                            double ux = T[0][2], uy = T[1][2], uz = T[2][2]; 
                            double nx = T[0][0], ny = T[1][0], nz = T[2][0]; 

                            // Crossbar direction vector is b
                            double bx = uy * nz - uz * ny;
                            double by = uz * nx - ux * nz;
                            double bz = ux * ny - uy * nx;
                            double blen = Math.sqrt(bx*bx + by*by + bz*bz);
                            if (blen > 1e-6) {
                                bx /= blen; by /= blen; bz /= blen;
                            }

                            double crossbarTilt = Math.toDegrees(Math.asin(bz));

                            if (Math.abs(crossbarTilt) <= 10.0) {
                                double fingerTilt = Math.toDegrees(Math.asin(uz));
                                System.out.printf("  q=[0.0, %.1f, %.1f, %.1f, %.1f, %.1f] TCP=(%.2f, %.2f, %.2f) fingerTilt=%.2f, crossbarTilt=%.2f\n",
                                        q2, q3, q4, q5, q6, x, y, z, fingerTilt, crossbarTilt);
                                count++;
                                if (count >= 15) return;
                            }
                        }
                    }
                }
            }
        }
        System.out.printf("Done. Found %d matches.\n", count);
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
