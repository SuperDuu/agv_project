package gui;

import java.util.ArrayList;
import java.util.List;

public class TestChairDemoValidity {
    public static void main(String[] args) {
        double sharedQ1 = 0.0;
        double[] homeRight = { sharedQ1, 0, 20, -35, 0, 0 };
        double[] homeLeft = { sharedQ1, 0, -20, 35, 0, 0 };
        
        double[] lowPickRight = { sharedQ1, 34.0, 50.0, -43.0, -90.0, 28.0 };
        double[] lowHoverRight = { sharedQ1, 36.0, 62.0, -57.0, -90.0, 28.0 };
        double[] lowApproachRight = { sharedQ1, 40.0, 80.0, -75.0, -90.0, 16.0 };

        double[] highPlaceRight = { sharedQ1, -66.0, 35.0, -39.0, -90.0, 3.0 };
        double[] highHoverRight = { sharedQ1, -70.0, 45.0, -51.0, -90.0, 11.0 };
        // Left arm keyframes
        double[] leftClear = { sharedQ1, -10, -45, 58, 18, 18 };
        double[] foldedHomeRight = { sharedQ1, 0, 120, -90, 0, -52 };
        
        List<double[][]> keyframes = new ArrayList<>();
        
        // 0. Initial folding of right arm, left arm moves to clear
        keyframes.add(new double[][] { homeRight, homeLeft });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { lowApproachRight, leftClear });
        
        // 1. Move to the low chair in one smooth approach
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        
        // 2. Pick low chair
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        keyframes.add(new double[][] { lowPickRight, leftClear });
        keyframes.add(new double[][] { lowPickRight, leftClear });
        keyframes.add(new double[][] { lowPickRight, leftClear });
        
        // 3. Lift low chair
        keyframes.add(new double[][] { lowPickRight, leftClear });
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        
        // 4. Travel to the far high chair through a compact folded pose near the body
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        keyframes.add(new double[][] { lowApproachRight, leftClear });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { highHoverRight, leftClear });
        
        // 5. Place low chair on high chair
        keyframes.add(new double[][] { highHoverRight, leftClear });
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        
        // 6. Release high chair
        keyframes.add(new double[][] { highHoverRight, leftClear });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { homeRight, homeLeft });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { highHoverRight, leftClear });
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        keyframes.add(new double[][] { highPlaceRight, leftClear });

        // 7. Retract right arm, left arm returns
        keyframes.add(new double[][] { highHoverRight, leftClear });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { lowApproachRight, leftClear });
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        keyframes.add(new double[][] { lowPickRight, leftClear });
        keyframes.add(new double[][] { lowPickRight, leftClear });
        keyframes.add(new double[][] { lowPickRight, leftClear });
        keyframes.add(new double[][] { lowApproachRight, leftClear });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { homeRight, homeLeft });

        // Test clearance
        ArmPanel panel = new ArmPanel(null); // Instantiate panel
        double[] lowPickCoord = panel.computeFK(sharedQ1, 34.0, 50.0, -43.0, -90.0, 28.0, true);
        double[] highPlaceCoord = panel.computeFK(sharedQ1, -66.0, 35.0, -39.0, -90.0, 3.0, true);
        double lowChairHeight = lowPickCoord[2] - 5.0;
        double highChairHeight = highPlaceCoord[2] - 5.0;
        double[] lowChairCenter = { lowPickCoord[0], lowPickCoord[1], 0.0 };
        double[] highChairCenter = { highPlaceCoord[0], highPlaceCoord[1], 0.0 };
        double margin = 3.0;

        System.out.println("Low Chair Center: (" + lowChairCenter[0] + ", " + lowChairCenter[1] + ")");
        System.out.println("High Chair Center: (" + highChairCenter[0] + ", " + highChairCenter[1] + ")");
        System.out.println("Low Chair Height: " + lowChairHeight);
        System.out.println("High Chair Height: " + highChairHeight);

        System.out.println("\nChecking chair clearance:");
        boolean clear = true;
        for (int i = 0; i < keyframes.size() - 1; i++) {
            double[][] kfStart = keyframes.get(i);
            double[][] kfEnd = keyframes.get(i + 1);
            int steps = 8;
            for (int s = 0; s <= steps; s++) {
                double t = (double) s / steps;
                double[] r = interpolate(kfStart[0], kfEnd[0], t);
                double[] l = interpolate(kfStart[1], kfEnd[1], t);
                
                boolean rClearLow = isArmClearOfChairBoxVerbose(r, true, lowChairCenter, lowChairHeight, margin, "Right", "Low");
                boolean lClearLow = isArmClearOfChairBoxVerbose(l, false, lowChairCenter, lowChairHeight, margin, "Left", "Low");
                boolean rClearHigh = isArmClearOfChairBoxVerbose(r, true, highChairCenter, highChairHeight, margin, "Right", "High");
                boolean lClearHigh = isArmClearOfChairBoxVerbose(l, false, highChairCenter, highChairHeight, margin, "Left", "High");
                
                if (!rClearLow || !lClearLow || !rClearHigh || !lClearHigh) {
                    System.out.printf("  [CLEARANCE ERROR] Segment %d->%d (step %d): R-Low:%b, L-Low:%b, R-High:%b, L-High:%b\n",
                            i, i+1, s, rClearLow, lClearLow, rClearHigh, lClearHigh);
                    clear = false;
                }
            }
        }
        if (clear) {
            System.out.println("  Clearance checks passed successfully!");
        } else {
            System.out.println("  Clearance violation detected!");
        }
    }

    private static double[] interpolate(double[] start, double[] end, double t) {
        double[] q = new double[start.length];
        for (int i = 0; i < q.length; i++) {
            q[i] = start[i] + t * (end[i] - start[i]);
        }
        return q;
    }

    private static boolean isArmClearOfChairBoxVerbose(double[] q, boolean isRight, double[] chairCenter,
            double chairHeight, double margin, String armName, String chairName) {
        double[][] pts = computeAllJoints3DForAngles(q, isRight);
        double minX = chairCenter[0] - 10.0 - margin;
        double maxX = chairCenter[0] + 10.0 + margin;
        double minY = chairCenter[1] - 10.0 - margin;
        double maxY = chairCenter[1] + 10.0 + margin;
        double minZ = 0.0 - margin;
        double maxZ = chairHeight + margin;

        double[] pt6 = pts[6];
        double[] pt7 = pts[7];
        double[] ptMid = { (pt6[0] + pt7[0])/2.0, (pt6[1] + pt7[1])/2.0, (pt6[2] + pt7[2])/2.0 };

        double[][] checkPts = { pt6, pt7, ptMid };
        String[] ptNames = { "Wrist", "TCP", "GripperMid" };
        for (int i = 0; i < checkPts.length; i++) {
            double[] pt = checkPts[i];
            if (pt[0] >= minX && pt[0] <= maxX
                    && pt[1] >= minY && pt[1] <= maxY
                    && pt[2] >= minZ && pt[2] <= maxZ) {
                System.out.printf("    [COLLISION DETAIL] %s arm %s at (%.2f, %.2f, %.2f) is inside %s Chair box [X: %.2f..%.2f, Y: %.2f..%.2f, Z: %.2f..%.2f]\n",
                        armName, ptNames[i], pt[0], pt[1], pt[2], chairName, minX, maxX, minY, maxY, minZ, maxZ);
                return false;
            }
        }
        return true;
    }

    private static double[][] computeAllJoints3DForAngles(double[] anglesDeg, boolean isRight) {
        double[][] pts = new double[8][3];
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
        pts[0] = new double[] { 0, 0, 0 };
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
