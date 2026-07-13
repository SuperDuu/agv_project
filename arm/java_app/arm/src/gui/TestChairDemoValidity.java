package gui;

import gui.ArmPanel.CollisionResult;
import kinematics.Kinematics;

public class TestChairDemoValidity {
    public static void main(String[] args) {
        // Run with system property to avoid opening JFrame if headless, but ArmPanel itself is a JPanel
        System.setProperty("java.awt.headless", "true");

        double sharedQ1 = 0.0;
        double[] leftClear = { sharedQ1, -10, -45, 58, 18, 18 };

        double[] homeRight = { sharedQ1, 0, 20, -35, 0, 0 };
        double[] homeLeft = { sharedQ1, 0, -20, 35, 0, 0 };
        
        double[] lowPickRight = { sharedQ1, -78.0, 20.0, -26.0, 0.0, -44.0 };
        double[] lowHoverRight = { sharedQ1, -82.0, 20.0, -26.0, 0.0, -44.0 };
        
        double[] highPlaceRight = { sharedQ1, -90.0, 152.0, -17.0, 0.0, -52.0 };
        double[] highHoverRight = { sharedQ1, -86.0, 152.0, -17.0, 0.0, -52.0 };
        
        double[] transferHighRight = highHoverRight.clone();
        double[] readyRight = highHoverRight.clone();
        double[] foldedHomeRight = { sharedQ1, 0.0, 120.0, -90.0, 0.0, -52.0 };
        double[] retreatRight = highHoverRight.clone();

        java.util.List<double[][]> keyframes = new java.util.ArrayList<>();
        keyframes.add(new double[][] { homeRight, homeLeft });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { readyRight, leftClear });
        
        // 1. Move to low chair
        keyframes.add(new double[][] { transferHighRight, leftClear });
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        
        // 2. Pick the object from low chair
        keyframes.add(new double[][] { lowPickRight, leftClear });
        
        // 3. Move to high chair
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        keyframes.add(new double[][] { transferHighRight, leftClear });
        keyframes.add(new double[][] { highHoverRight, leftClear });
        
        // 4. Place the object on high chair
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        
        // 5. Retract to ready position and wait
        keyframes.add(new double[][] { highHoverRight, leftClear });
        keyframes.add(new double[][] { retreatRight, leftClear });
        keyframes.add(new double[][] { readyRight, leftClear });
        
        // 6. Return to high chair to pick it up again
        keyframes.add(new double[][] { transferHighRight, leftClear });
        keyframes.add(new double[][] { highHoverRight, leftClear });
        
        // 7. Pick the object from high chair
        keyframes.add(new double[][] { highPlaceRight, leftClear });
        
        // 8. Move back to low chair
        keyframes.add(new double[][] { highHoverRight, leftClear });
        keyframes.add(new double[][] { transferHighRight, leftClear });
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        
        // 9. Place the object back on low chair
        keyframes.add(new double[][] { lowPickRight, leftClear });
        
        // 10. Retract and go home
        keyframes.add(new double[][] { lowHoverRight, leftClear });
        keyframes.add(new double[][] { transferHighRight, leftClear });
        keyframes.add(new double[][] { readyRight, leftClear });
        keyframes.add(new double[][] { foldedHomeRight, leftClear });
        keyframes.add(new double[][] { homeRight, homeLeft });

        System.out.println("Checking collision on all frames:");
        boolean collisionFree = true;
        int samplesPerSegment = 8;

        for (int i = 0; i < keyframes.size(); i++) {
            CollisionResult collision = ArmPanel.diagnoseCollision(keyframes.get(i)[0], keyframes.get(i)[1]);
            if (!collision.free) {
                System.out.printf("  [COLLISION ERROR] Keyframe %d: %s\n", i, collision);
                collisionFree = false;
            }
        }

        for (int i = 0; i < keyframes.size() - 1; i++) {
            for (int step = 1; step < samplesPerSegment; step++) {
                double t = (double) step / samplesPerSegment;
                double[] r = interpolate(keyframes.get(i)[0], keyframes.get(i+1)[0], t);
                double[] l = interpolate(keyframes.get(i)[1], keyframes.get(i+1)[1], t);
                CollisionResult collision = ArmPanel.diagnoseCollision(r, l);
                if (!collision.free) {
                    System.out.printf("  [COLLISION ERROR] Segment %d -> %d (step %d): %s\n", i, i+1, step, collision);
                    collisionFree = false;
                }
            }
        }

        if (collisionFree) {
            System.out.println("  All keyframes and interpolated transitions are collision-free!");
        } else {
            System.out.println("  Collision detected!");
        }

        // Test clearance
        ArmPanel panel = new ArmPanel(null); // Instantiate panel
        double[] lowPickCoord = panel.computeFK(sharedQ1, -78.0, 20.0, -26.0, 0.0, -44.0, true);
        double[] highPlaceCoord = panel.computeFK(sharedQ1, -90.0, 152.0, -17.0, 0.0, -52.0, true);
        double lowChairHeight = lowPickCoord[2] - 5.0;
        double highChairHeight = highPlaceCoord[2] - 5.0;
        double[] lowChairCenter = { lowPickCoord[0], lowPickCoord[1], 0.0 };
        double[] highChairCenter = { highPlaceCoord[0], highPlaceCoord[1], 0.0 };

        System.out.println("\nChecking chair clearance:");
        boolean clearanceFree = true;
        for (int i = 0; i < keyframes.size(); i++) {
            if (!isChairFrameClear(keyframes.get(i), lowChairCenter, lowChairHeight, highChairCenter, highChairHeight)) {
                System.out.printf("  [CLEARANCE ERROR] Keyframe %d violates chair clearance!\n", i);
                clearanceFree = false;
            }
        }

        for (int i = 0; i < keyframes.size() - 1; i++) {
            for (int step = 1; step < samplesPerSegment; step++) {
                double t = (double) step / samplesPerSegment;
                double[] r = interpolate(keyframes.get(i)[0], keyframes.get(i+1)[0], t);
                double[] l = interpolate(keyframes.get(i)[1], keyframes.get(i+1)[1], t);
                double[][] frame = { r, l };
                if (!isChairFrameClear(frame, lowChairCenter, lowChairHeight, highChairCenter, highChairHeight)) {
                    System.out.printf("  [CLEARANCE ERROR] Segment %d -> %d (step %d) violates chair clearance!\n", i, i+1, step);
                    clearanceFree = false;
                }
            }
        }

        if (clearanceFree) {
            System.out.println("  All keyframes and interpolated transitions clear the chairs successfully!");
        } else {
            System.out.println("  Clearance violation detected!");
        }
    }

    private static double[] interpolate(double[] start, double[] end, double t) {
        double[] res = new double[6];
        for (int i = 0; i < 6; i++) {
            res[i] = start[i] + t * (end[i] - start[i]);
        }
        return res;
    }

    private static boolean isChairFrameClear(double[][] frame, double[] lowChairCenter, double lowChairHeight,
            double[] highChairCenter, double highChairHeight) {
        double margin = 3.0;
        return isArmClearOfChairBox(frame[0], true, lowChairCenter, lowChairHeight, margin)
                && isArmClearOfChairBox(frame[1], false, lowChairCenter, lowChairHeight, margin)
                && isArmClearOfChairBox(frame[0], true, highChairCenter, highChairHeight, margin)
                && isArmClearOfChairBox(frame[1], false, highChairCenter, highChairHeight, margin);
    }

    private static boolean isArmClearOfChairBox(double[] q, boolean isRight, double[] chairCenter,
            double chairHeight, double margin) {
        double[][] pts = ArmPanel.computeAllJoints3DForAngles(q, isRight);
        double minX = chairCenter[0] - 10.0 - margin; // ArmPanel.CHAIR_DEMO_HALF_X is 10.0
        double maxX = chairCenter[0] + 10.0 + margin;
        double minY = chairCenter[1] - 10.0 - margin; // ArmPanel.CHAIR_DEMO_HALF_Y is 10.0
        double maxY = chairCenter[1] + 10.0 + margin;
        double minZ = 0.0 - margin;
        double maxZ = chairHeight + margin;

        double[] pt6 = pts[6];
        double[] pt7 = pts[7];
        double[] ptMid = { (pt6[0] + pt7[0])/2.0, (pt6[1] + pt7[1])/2.0, (pt6[2] + pt7[2])/2.0 };

        double[][] checkPts = { pt6, pt7, ptMid };
        for (double[] pt : checkPts) {
            if (pt[0] >= minX && pt[0] <= maxX
                    && pt[1] >= minY && pt[1] <= maxY
                    && pt[2] >= minZ && pt[2] <= maxZ) {
                return false;
            }
        }
        return true;
    }
}
