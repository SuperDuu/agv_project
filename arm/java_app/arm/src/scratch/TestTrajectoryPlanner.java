package scratch;

import java.util.ArrayList;
import java.util.List;
import kinematics.TrajectoryPlanner;

public class TestTrajectoryPlanner {
    public static void main(String[] args) {
        System.out.println("=== Testing TrajectoryPlanner ===");

        // 1. Generate a mock path representing a joint-space movement with gaps
        List<double[]> rawPath = new ArrayList<>();
        
        // Let's create 10 waypoints for Joint 1 moving from 0 to 90 degrees
        for (int i = 0; i <= 10; i++) {
            if (i == 3 || i == 4 || i == 7) {
                // Insert some nulls (simulating IK failures/gaps)
                rawPath.add(null);
            } else {
                double[] q = new double[6];
                q[0] = i * 9.0; // Joint 1 goes 0 -> 9.0 -> 18.0 ... -> 90.0
                q[1] = i * 2.0;
                q[2] = -i * 1.5;
                q[3] = -40.0;
                q[4] = 10.0;
                q[5] = i * 5.0;
                rawPath.add(q);
            }
        }

        System.out.println("Raw Path size: " + rawPath.size());
        int nullCount = 0;
        for (double[] q : rawPath) {
            if (q == null) nullCount++;
        }
        System.out.println("Gaps count: " + nullCount);

        // 2. Test Gap Filling
        double[] defaultQ = new double[]{0, 0, 0, -40, 10, 0};
        List<double[]> filledPath = TrajectoryPlanner.fillGaps(rawPath, defaultQ);
        System.out.println("Filled Path size: " + filledPath.size());
        for (int i = 0; i < filledPath.size(); i++) {
            double[] q = filledPath.get(i);
            System.out.printf("  Point %2d: Q1=%.2f, Q2=%.2f, Q3=%.2f, Q6=%.2f\n", i, q[0], q[1], q[2], q[5]);
        }

        // 3. Test planTrajectory (Option A: zero acceleration at via points)
        double maxVel = 180.0; // deg/s
        double maxAcc = 360.0; // deg/s^2
        double dt = 0.030; // 30ms
        
        System.out.println("\n--- Testing Trajectory Generation Option A (Zero via-accel) ---");
        List<double[]> trajA = TrajectoryPlanner.planTrajectory(filledPath, maxVel, maxAcc, false, dt);
        System.out.println("Option A Trajectory points: " + trajA.size());
        System.out.println("Option A Trajectory duration: " + (trajA.size() * dt) + " seconds");
        verifyLimits(trajA, maxVel, maxAcc, dt);

        // 4. Test planTrajectory (Option B: global C2 continuity)
        System.out.println("\n--- Testing Trajectory Generation Option B (Global C2 continuity) ---");
        List<double[]> trajB = TrajectoryPlanner.planTrajectory(filledPath, maxVel, maxAcc, true, dt);
        System.out.println("Option B Trajectory points: " + trajB.size());
        System.out.println("Option B Trajectory duration: " + (trajB.size() * dt) + " seconds");
        verifyLimits(trajB, maxVel, maxAcc, dt);

        System.out.println("=== TrajectoryPlanner Tests Completed Successfully ===");
    }

    private static void verifyLimits(List<double[]> traj, double maxVel, double maxAcc, double dt) {
        double maxObservedVel = 0.0;
        double maxObservedAcc = 0.0;
        
        for (int i = 1; i < traj.size(); i++) {
            double[] qPrev = traj.get(i - 1);
            double[] qCurr = traj.get(i);
            
            for (int j = 0; j < 6; j++) {
                double diff = qCurr[j] - qPrev[j];
                while (diff > 180) diff -= 360;
                while (diff < -180) diff += 360;
                
                double vel = Math.abs(diff / dt);
                if (vel > maxObservedVel) maxObservedVel = vel;
            }
        }
        
        for (int i = 2; i < traj.size(); i++) {
            double[] qPrev2 = traj.get(i - 2);
            double[] qPrev1 = traj.get(i - 1);
            double[] qCurr = traj.get(i);
            
            for (int j = 0; j < 6; j++) {
                double diff1 = qPrev1[j] - qPrev2[j];
                while (diff1 > 180) diff1 -= 360;
                while (diff1 < -180) diff1 += 360;
                double v1 = diff1 / dt;
                
                double diff2 = qCurr[j] - qPrev1[j];
                while (diff2 > 180) diff2 -= 360;
                while (diff2 < -180) diff2 += 360;
                double v2 = diff2 / dt;
                
                double acc = Math.abs((v2 - v1) / dt);
                if (acc > maxObservedAcc) maxObservedAcc = acc;
            }
        }
        
        System.out.printf("  Max limits: Velocity = %.1f deg/s, Acceleration = %.1f deg/s^2\n", maxVel, maxAcc);
        System.out.printf("  Observed  : Max Vel  = %.3f deg/s, Max Acc      = %.3f deg/s^2\n", maxObservedVel, maxObservedAcc);
        
        if (maxObservedVel > maxVel + 1e-3) {
            System.err.println("  [ERROR] Velocity limit violated!");
        } else if (maxObservedAcc > maxAcc + 1e-3) {
            System.err.println("  [ERROR] Acceleration limit violated!");
        } else {
            System.out.println("  [SUCCESS] All velocity and acceleration limits are satisfied.");
        }
    }
}
