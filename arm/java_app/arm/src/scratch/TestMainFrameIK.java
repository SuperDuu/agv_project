package scratch;

import gui.MainFrame;
import java.util.Arrays;

public class TestMainFrameIK {
    public static void main(String[] args) {
        // Instantiate MainFrame (it will initialize the UI but we can just use its IK methods)
        // To avoid showing the GUI, we don't call setVisible(true)
        System.out.println("Instantiating MainFrame...");
        MainFrame frame = new MainFrame();
        
        // Target: [38.41, 32.65, 80.0]
        double px = 38.41, py = 32.65, pz = 80.0;
        
        System.out.println("=== Testing MainFrame IK for Right Arm ===");
        frame.isRightArmSelected = true;
        
        // Let's test with alpha optimization (which is what run does under the hood)
        double[] q = frame.solveIKSmart(px, py, pz, "Elbow Up");
        if (q != null) {
            System.out.printf("FOUND OPTIMAL SOLUTION: q = %s\n", Arrays.toString(q));
            System.out.printf("  q1 = %.2f (Waist)\n", q[0]);
            System.out.printf("  q2 = %.2f (Shoulder Pitch - SHOULD BE NON-ZERO!)\n", q[1]);
            System.out.printf("  q3 = %.2f (Shoulder Roll)\n", q[2]);
            System.out.printf("  q4 = %.2f (Elbow Pitch - MUST BE NEGATIVE!)\n", q[3]);
        } else {
            System.out.println("FAILED to find any valid solution!");
        }
        
        System.exit(0);
    }
}
