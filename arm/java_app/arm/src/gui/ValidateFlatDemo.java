package gui;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Locale;

public class ValidateFlatDemo {
    public static void main(String[] args) {
        try {
            System.loadLibrary("kinematics_jni");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Could not load JNI library: " + e.getMessage());
            System.exit(1);
        }

        try {
            System.out.println("Instantiating MainFrame...");
            MainFrame mf = new MainFrame();
            
            // Print the key poses
            double flatA = -15.0;
            double[][] poses = {
                new double[] { 0, 0, 20, -35, 0, 0 }, // HOME
                makeRightFlatGripperPose(35.0, 30.0, flatA, 80.0), // lowPick
                makeRightFlatGripperPose(35.0, 80.0, flatA, 80.0), // lowHover
                makeRightFlatGripperPose(35.0, 80.0, flatA, -80.0), // lowExit
                makeRightFlatGripperPose(0.0, 80.0, flatA, -80.0), // centerExit
                makeRightFlatGripperPose(-45.0, 80.0, flatA, -80.0), // highPlace
                makeRightFlatGripperPose(0.0, 80.0, flatA, 0.0) // flatHome
            };
            String[] names = {
                "HOME", "lowPick", "lowHover", "lowExit", "centerExit", "highPlace", "flatHome"
            };
            
            System.out.println("--- Poses info ---");
            for (int i = 0; i < poses.length; i++) {
                double[] q = poses[i];
                double[] tcp = mf.armPanel.computeFK(q[0], q[1], q[2], q[3], q[4], q[5], true);
                System.out.printf(Locale.US, "%s: joints=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] TCP=[%.4f, %.4f, %.4f]%n",
                        names[i], q[0], q[1], q[2], q[3], q[4], q[5], tcp[0], tcp[1], tcp[2]);
            }
            
            System.out.println("Invoking buildFlatQ1ChairTransferDemo via reflection...");
            Method method = MainFrame.class.getDeclaredMethod("buildFlatQ1ChairTransferDemo");
            method.setAccessible(true);
            Object plan = method.invoke(mf);
            if (plan != null) {
                System.out.println("PLAN_OK");
                Field framesField = plan.getClass().getDeclaredField("frames");
                framesField.setAccessible(true);
                java.util.List<?> frames = (java.util.List<?>) framesField.get(plan);
                System.out.println("frames=" + frames.size());
                
                // Let's print chair scene details
                Field sceneField = plan.getClass().getDeclaredField("chairScene");
                sceneField.setAccessible(true);
                Object scene = sceneField.get(plan);
                if (scene != null) {
                    Field lowCenterF = scene.getClass().getDeclaredField("lowChairCenter");
                    Field lowHeightF = scene.getClass().getDeclaredField("lowChairHeight");
                    Field highCenterF = scene.getClass().getDeclaredField("highChairCenter");
                    Field highHeightF = scene.getClass().getDeclaredField("highChairHeight");
                    lowCenterF.setAccessible(true);
                    lowHeightF.setAccessible(true);
                    highCenterF.setAccessible(true);
                    highHeightF.setAccessible(true);
                    double[] lowCenter = (double[]) lowCenterF.get(scene);
                    double lowHeight = (double) lowHeightF.get(scene);
                    double[] highCenter = (double[]) highCenterF.get(scene);
                    double highHeight = (double) highHeightF.get(scene);
                    System.out.printf(Locale.US, "Low Chair: Center=[%.4f, %.4f, %.4f] Height=%.4f%n",
                            lowCenter[0], lowCenter[1], lowCenter[2], lowHeight);
                    System.out.printf(Locale.US, "High Chair: Center=[%.4f, %.4f, %.4f] Height=%.4f%n",
                            highCenter[0], highCenter[1], highCenter[2], highHeight);
                }
            } else {
                System.out.println("PLAN_FAILED (null plan)");
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static double[] makeRightFlatGripperPose(double q1, double q3, double aDeg, double q5) {
        double aRad = Math.toRadians(aDeg);
        double q5Rad = Math.toRadians(q5);
        double q2 = Math.toDegrees(Math.atan(-Math.tan(q5Rad) * Math.sin(aRad)));
        double q6 = Math.toDegrees(Math.atan(-Math.cos(q5Rad) / Math.tan(aRad)));
        if (q6 < 0.0) {
            q6 += 180.0;
        }
        return new double[] { q1, q2, q3, aDeg - q3, q5, q6 };
    }
}
