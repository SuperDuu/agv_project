package gui;

public class TestDemoPlayback {
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
            
            // Start the Flat Q1 Chair Demo
            System.out.println("Running flat Q1 chair transfer showcase...");
            mf.runFlatQ1ChairTransferShowcase();
            
            // Monitor status
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 15000) {
                Thread.sleep(100);
                System.out.printf("Time: %d ms | active: %b | J3_Right: cur=%.2f, tgt=%.2f | J3_Left: cur=%.2f, tgt=%.2f%n",
                        System.currentTimeMillis() - startTime,
                        mf.dualDemoActive,
                        mf.anglesRight[2], mf.targetAnglesRight[2],
                        mf.anglesLeft[2], mf.targetAnglesLeft[2]
                );
                if (!mf.dualDemoActive && (System.currentTimeMillis() - startTime > 1000)) {
                    System.out.println("Demo ended.");
                    break;
                }
            }
            System.exit(0);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
    }
}
