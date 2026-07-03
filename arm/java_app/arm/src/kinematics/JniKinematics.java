package kinematics;

import java.io.File;

public class JniKinematics {
    private static boolean libraryLoaded = false;

    static {
        try {
            System.loadLibrary("kinematics_jni");
            libraryLoaded = true;
        } catch (UnsatisfiedLinkError e) {
            try {
                String libName = System.mapLibraryName("kinematics_jni");
                File libFile = new File("lib/" + libName);
                if (libFile.exists()) {
                    System.load(libFile.getAbsolutePath());
                    libraryLoaded = true;
                } else {
                    libFile = new File(libName);
                    if (libFile.exists()) {
                        System.load(libFile.getAbsolutePath());
                        libraryLoaded = true;
                    }
                }
            } catch (UnsatisfiedLinkError ex) {
                System.err.println("Warning: kinematics_jni library could not be loaded. Falling back to Java solver.");
            }
        }
    }

    public static boolean isLoaded() {
        return libraryLoaded;
    }

    /**
     * Native IK Solver.
     * Calls C++ JNI library which executes IKFast / Orocos KDL.
     * Returns double[6] angles in degrees, or null if no solution.
     */
    public static native double[] solveIKNative(double px, double py, double pz, double[][] R_target, double[] qInitRad, boolean isRight);
}
