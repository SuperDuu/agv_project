package gui;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

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
            } else {
                System.out.println("PLAN_FAILED (null plan)");
            }
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
    }
}
