import kinematics.RobotTransmission;

public class FindVerticalLiftPairWithQ5 {
    public static void main(String[] args) {
        System.out.println("Searching for vertical lift pairs for High Chair (Q5 = ±90, Q6 ∈ [-60, 60]):");
        searchPair(130.0, 134.0); 
        
        System.out.println("\nSearching for vertical lift pairs for Low Chair (Q5 = ±90, Q6 ∈ [-60, 60]):");
        searchPair(118.0, 122.0); 
    }

    private static void searchPair(double targetZPlace, double minZHover) {
        java.util.List<Config> list = new java.util.ArrayList<>();
        
        // Scan joint space with step 2
        for (double q2 = -90.0; q2 <= 0.0; q2 += 2.0) {
            for (double q3 = 20.0; q3 <= 165.0; q3 += 2.0) {
                for (double q4 = -95.0; q4 <= 20.0; q4 += 2.0) {
                    double[] qAct = RobotTransmission.jointToActuator(q3, q4, true);
                    double diff = qAct[0] - qAct[1];
                    if (Math.abs(diff) < 5.0 || Math.abs(diff) > 90.0) {
                        continue;
                    }

                    for (double q5 : new double[] { -90.0, 90.0 }) {
                        for (double q6 = -60.0; q6 <= 60.0; q6 += 2.0) { // Restrict q6 to [-60, 60]!
                            double[] q = { 0.0, q2, q3, q4, q5, q6 };
                            double[][] T = computeMDHMatrix(q, true);
                            
                            double x = T[0][3];
                            double y = T[1][3];
                            double z = T[2][3];
                            
                            if (x < 45.0 || x > 75.0 || Math.abs(y) > 60.0) {
                                continue;
                            }

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

                            if (Math.abs(fingerTilt) <= 10.0 && Math.abs(crossbarTilt) <= 10.0) {
                                list.add(new Config(q, x, y, z, fingerTilt, crossbarTilt, diff));
                            }
                        }
                    }
                }
            }
        }

        System.out.printf("Found %d total horizontal configurations in workspace.\n", list.size());

        double bestScore = 9999.0;
        Config bestP = null;
        Config bestH = null;

        for (Config p : list) {
            if (Math.abs(p.z - targetZPlace) > 0.6) continue;
            for (Config h : list) {
                if (h.z < p.z + 3.5) continue; // Minimum lift 3.5 cm
                if (Math.abs(h.q[4] - p.q[4]) > 0.1) continue; // Q5 must not flip between pick and hover!
                
                double dx = h.x - p.x;
                double dy = h.y - p.y;
                if (Math.sqrt(dx*dx + dy*dy) > 1.2) continue; // Vertically aligned within 1.2 cm
                
                double d2 = h.q[1] - p.q[1];
                double d3 = h.q[2] - p.q[2];
                double d4 = h.q[3] - p.q[3];
                double d6 = h.q[5] - p.q[5];
                double jointDist = Math.sqrt(d2*d2 + d3*d3 + d4*d4 + d6*d6);
                
                if (jointDist < bestScore) {
                    bestScore = jointDist;
                    bestP = p;
                    bestH = h;
                }
            }
        }

        if (bestP != null) {
            System.out.printf("Best vertical pair found (joint distance = %.2f):\n", bestScore);
            System.out.printf("  Place/Pick: q=[0.0, %.1f, %.1f, %.1f, %.1f, %.1f] TCP=(%.2f, %.2f, %.2f) fingerTilt=%.2f, crossbarTilt=%.2f\n",
                    bestP.q[1], bestP.q[2], bestP.q[3], bestP.q[4], bestP.q[5], bestP.x, bestP.y, bestP.z, bestP.fingerTilt, bestP.crossbarTilt);
            System.out.printf("  Hover:      q=[0.0, %.1f, %.1f, %.1f, %.1f, %.1f] TCP=(%.2f, %.2f, %.2f) fingerTilt=%.2f, crossbarTilt=%.2f\n",
                    bestH.q[1], bestH.q[2], bestH.q[3], bestH.q[4], bestH.q[5], bestH.x, bestH.y, bestH.z, bestH.fingerTilt, bestH.crossbarTilt);
        } else {
            System.out.println("No matching vertical pair found!");
        }
    }

    private static class Config {
        double[] q;
        double x, y, z;
        double fingerTilt, crossbarTilt;
        double diff;
        Config(double[] q, double x, double y, double z, double ft, double ct, double d) {
            this.q = q.clone();
            this.x = x; this.y = y; this.z = z;
            this.fingerTilt = ft; this.crossbarTilt = ct;
            this.diff = d;
        }
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
