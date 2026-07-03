package scratch;

import gui.MainFrame;
import kinematics.Kinematics;
import java.lang.reflect.Method;
import java.util.List;
import java.util.ArrayList;

public class TestDescartes {
    public static void main(String[] args) throws Exception {
        System.out.println("=== Testing Descartes DP Planner (Replaying Doc 5 Points 31-33) ===");
        
        MainFrame frame = new MainFrame();
        
        List<double[]> path = new ArrayList<>();
        // Inject a sequence of points. We'll start with 30 and go up to 34.
        path.add(new double[]{38.00, -22.50, 100.00}); // Pt 30
        path.add(new double[]{38.31, -22.06, 100.00}); // Pt 31
        path.add(new double[]{38.66, -21.81, 100.00}); // Pt 32
        path.add(new double[]{41.47, -18.81, 100.00}); // Pt 33 (OUTSIDE)
        path.add(new double[]{42.26, -17.42, 100.00}); // Pt 34 (OUTSIDE)
        path.add(new double[]{40.00, -10.00, 100.00}); // Pt 35 (BACK INSIDE)

        Method planMethod = MainFrame.class.getDeclaredMethod("planDescartesTrajectory", List.class, boolean.class);
        planMethod.setAccessible(true);
        
        System.out.println("Invoking planDescartesTrajectory...");
        List<double[]> result = (List<double[]>) planMethod.invoke(frame, path, true);
        
        System.out.println("\n=== Result Trajectory ===");
        double[] qRef = null;
        for (int i = 0; i < result.size(); i++) {
            double[] q = result.get(i);
            if (q == null) {
                System.out.printf("Node %d: NULL (Unreachable)\n", i);
            } else {
                double maxJ = 0;
                if (qRef != null) {
                    for (int j = 0; j < 6; j++) {
                        double d = Math.abs(q[j] - qRef[j]);
                        while (d > 180) d -= 360;
                        while (d < -180) d += 360;
                        d = Math.abs(d);
                        if (d > maxJ) maxJ = d;
                    }
                }
                
                // Calculate pos error
                double[][] T_fk = Kinematics.computeFKMatrix(q, true);
                double px = T_fk[0][3], py = T_fk[1][3], pz = T_fk[2][3];
                System.out.printf("Node %d: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] | Jump=%.1f | FK=(%.1f, %.1f, %.1f)\n", 
                    i, q[0], q[1], q[2], q[3], q[4], q[5], maxJ, px, py, pz);
                
                qRef = q;
            }
        }
        
        System.out.println("\n=== Test Completed Successfully ===");
        System.exit(0);
    }
}
