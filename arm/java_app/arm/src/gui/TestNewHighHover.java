package gui;

import kinematics.Kinematics;
import java.util.ArrayList;
import java.util.List;

public class TestNewHighHover {
    private static class ChairDemoScene {
        double[] lowChairCenter;
        double lowChairHeight;
        double[] highChairCenter;
        double highChairHeight;

        ChairDemoScene(double[] lowChairCenter, double lowChairHeight,
                       double[] highChairCenter, double highChairHeight) {
            this.lowChairCenter = lowChairCenter;
            this.lowChairHeight = lowChairHeight;
            this.highChairCenter = highChairCenter;
            this.highChairHeight = highChairHeight;
        }
    }

    public static void main(String[] args) {
        // Load the JNI library
        try {
            System.loadLibrary("kinematics_jni");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Could not load JNI library: " + e.getMessage());
            return;
        }

        double sharedQ1 = 0.0;
        double[] leftClear = { sharedQ1, -10, -45, 58, 18, 18 };

        double[] homeRight = { sharedQ1, 0, 20, -35, 0, 0 };
        double[] homeLeft = { sharedQ1, 0, -20, 35, 0, 0 };
        
        // Define our new lower configurations (Option B)!
        double[] lowPickRight = { sharedQ1, 20.0, 48.0, -17.0, 40.0, 10.0 };
        double[] lowHoverRight = { sharedQ1, 35.0, 48.0, -17.0, 40.0, 10.0 };
        
        double[] highPlaceRight = { sharedQ1, -64.0, 42.0, -19.0, -35.0, -45.0 };
        double[] highHoverRight = { sharedQ1, -66.0, 42.0, -19.0, -35.0, -45.0 };
        
        double[] transferHighRight = { sharedQ1, -90.0, 120.0, -90.0, 40.0, 10.0 };
        double[] readyRight = { sharedQ1, -90.0, 120.0, -90.0, 40.0, 10.0 };
        double[] foldedHomeRight = { sharedQ1, 0.0, 120.0, -90.0, 40.0, 10.0 };

        List<double[][]> keyframes = new ArrayList<>();
        keyframes.add(new double[][] { homeRight, homeLeft });                      // 0
        keyframes.add(new double[][] { foldedHomeRight, leftClear });               // 1: fold in-place
        keyframes.add(new double[][] { readyRight, leftClear });                    // 2: lift to ready
        keyframes.add(new double[][] { transferHighRight, leftClear });             // 3: ready to transfer
        keyframes.add(new double[][] { lowHoverRight, leftClear });                 // 4: descend to low hover
        keyframes.add(new double[][] { lowPickRight, leftClear });                  // 5: pick
        keyframes.add(new double[][] { lowHoverRight, leftClear });                 // 6
        keyframes.add(new double[][] { transferHighRight, leftClear });             // 7: lift to transfer
        keyframes.add(new double[][] { highHoverRight, leftClear });                // 8: move to high hover
        keyframes.add(new double[][] { highPlaceRight, leftClear });                // 9: place
        keyframes.add(new double[][] { highHoverRight, leftClear });                // 10
        keyframes.add(new double[][] { transferHighRight, leftClear });             // 11: retreat to transfer
        keyframes.add(new double[][] { readyRight, leftClear });                    // 12: wait
        keyframes.add(new double[][] { transferHighRight, leftClear });             // 13
        keyframes.add(new double[][] { highHoverRight, leftClear });                // 14
        keyframes.add(new double[][] { highPlaceRight, leftClear });                // 15: pick back
        keyframes.add(new double[][] { highHoverRight, leftClear });                // 16
        keyframes.add(new double[][] { transferHighRight, leftClear });             // 17
        keyframes.add(new double[][] { lowHoverRight, leftClear });                 // 18
        keyframes.add(new double[][] { lowPickRight, leftClear });                  // 19: place back
        keyframes.add(new double[][] { lowHoverRight, leftClear });                 // 20
        keyframes.add(new double[][] { transferHighRight, leftClear });             // 21
        keyframes.add(new double[][] { readyRight, leftClear });                    // 22
        keyframes.add(new double[][] { foldedHomeRight, leftClear });               // 23: lower elbow
        keyframes.add(new double[][] { homeRight, homeLeft });                      // 24: unfold home

        int samples = 8;
        boolean allOk = true;
        
        // Define Chair Scene
        double[] lowPickCoord = computeFK(lowPickRight);
        double[] highPlaceCoord = computeFK(highPlaceRight);
        double lowChairHeight = lowPickCoord[2] - 5.0; // 80.00 cm
        double highChairHeight = highPlaceCoord[2] - 5.0; // 100.14 cm
        System.out.printf("Low Chair Height: %.2f, High Chair Height: %.2f\n", lowChairHeight, highChairHeight);
        ChairDemoScene scene = new ChairDemoScene(
                new double[] { lowPickCoord[0], lowPickCoord[1], 0.0 }, lowChairHeight,
                new double[] { highPlaceCoord[0], highPlaceCoord[1], 0.0 }, highChairHeight);

        for (int i = 0; i < keyframes.size() - 1; i++) {
            for (int step = 0; step < samples; step++) {
                double t = step / (double) samples;
                double[][] interp = interpolate(keyframes.get(i), keyframes.get(i+1), t);

                // Self collision check
                ArmPanel.CollisionResult collision = ArmPanel.diagnoseCollision(interp[0], interp[1]);
                if (!collision.free) {
                    System.out.printf("SELF-COLLISION at segment %d step %d: %s\n", i, step, collision);
                    allOk = false;
                }

                // Chair collision check
                if (!isChairFrameClear(interp, scene)) {
                    System.out.printf("CHAIR-COLLISION at segment %d step %d\n", i, step);
                    allOk = false;
                }
            }
        }

        if (allOk) {
            System.out.println("SUCCESS! Entire trajectory has ZERO collisions!");
        } else {
            System.out.println("FAILED!");
        }
    }

    private static boolean isChairFrameClear(double[][] frame, ChairDemoScene scene) {
        double margin = 3.0;
        boolean ok = true;
        if (!isArmClearOfChairBox(frame[0], true, scene.lowChairCenter, scene.lowChairHeight, margin, "RIGHT arm with LOW chair")) ok = false;
        if (!isArmClearOfChairBox(frame[1], false, scene.lowChairCenter, scene.lowChairHeight, margin, "LEFT arm with LOW chair")) ok = false;
        if (!isArmClearOfChairBox(frame[0], true, scene.highChairCenter, scene.highChairHeight, margin, "RIGHT arm with HIGH chair")) ok = false;
        if (!isArmClearOfChairBox(frame[1], false, scene.highChairCenter, scene.highChairHeight, margin, "LEFT arm with HIGH chair")) ok = false;
        return ok;
    }

    private static boolean isArmClearOfChairBox(double[] q, boolean isRight, double[] chairCenter,
            double chairHeight, double margin, String label) {
        double[][] pts = ArmPanel.computeAllJoints3DForAngles(q, isRight);
        double minX = chairCenter[0] - ArmPanel.CHAIR_DEMO_HALF_X - margin;
        double maxX = chairCenter[0] + ArmPanel.CHAIR_DEMO_HALF_X + margin;
        double minY = chairCenter[1] - ArmPanel.CHAIR_DEMO_HALF_Y - margin;
        double maxY = chairCenter[1] + ArmPanel.CHAIR_DEMO_HALF_Y + margin;
        double minZ = 0.0 - margin;
        double maxZ = chairHeight + margin;

        double[] pt6 = pts[6];
        double[] pt7 = pts[7];
        double[] ptMid = { (pt6[0] + pt7[0])/2.0, (pt6[1] + pt7[1])/2.0, (pt6[2] + pt7[2])/2.0 };

        double[][] checkPts = { pt6, pt7, ptMid };
        for (int pIdx = 0; pIdx < checkPts.length; pIdx++) {
            double[] pt = checkPts[pIdx];
            if (pt[0] >= minX && pt[0] <= maxX
                    && pt[1] >= minY && pt[1] <= maxY
                    && pt[2] >= minZ && pt[2] <= maxZ) {
                System.out.printf("  [COLLISION] %s at pt %d: (%.2f, %.2f, %.2f) limits: X=[%.2f, %.2f], Y=[%.2f, %.2f], Z=[%.2f, %.2f]\n",
                        label, pIdx, pt[0], pt[1], pt[2], minX, maxX, minY, maxY, minZ, maxZ);
                return false;
            }
        }
        return true;
    }

    private static double[] computeFK(double[] q) {
        double[] qRad = new double[6];
        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
        double[][] T = Kinematics.computeFKMatrix(qRad, true);
        return new double[] { T[0][3], T[1][3], T[2][3] };
    }

    private static double[][] interpolate(double[][] f1, double[][] f2, double t) {
        double[][] res = new double[2][6];
        for (int arm = 0; arm < 2; arm++) {
            for (int joint = 0; joint < 6; joint++) {
                res[arm][joint] = f1[arm][joint] + (f2[arm][joint] - f1[arm][joint]) * t;
            }
        }
        return res;
    }
}
