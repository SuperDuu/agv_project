package scratch;

import gui.MainFrame;
import kinematics.Kinematics;
import java.lang.reflect.Method;
import java.lang.reflect.Field;
import java.util.List;

public class TestBranchTransition {
    public static void main(String[] args) throws Exception {
        System.out.println("=== Testing Branch Transition (Replaying Doc 5 Points 31-33) ===");
        
        // 1. Khởi tạo MainFrame để mô phỏng môi trường
        MainFrame frame = new MainFrame();
        
        // 2. Thiết lập trạng thái bắt đầu (mô phỏng trạng thái trước khi điểm 31 fail)
        // Đây là giá trị ví dụ gần đúng vùng fail trong log cũ
        setField(frame, "trajectoryLastAlpha", 0.0);
        setField(frame, "trajectoryLockedCfg", "+");
        setField(frame, "trajectoryLastYawOffset", 0.0);
        setField(frame, "isRightArmSelected", true);
        
        // Tọa độ từ Log gốc (Doc 5) cho điểm 31-33
        double[][] targets = {
            {38.31, -22.06, 100.00}, // Điểm 31 (FAIL trong doc 5)
            {38.66, -21.81, 100.00}, // Điểm 32 (FAIL)
            {41.47, -18.81, 100.00}, // Điểm 33 (FAIL)
            {42.26, -17.42, 100.00}  // Điểm 34 (FAIL)
        };
        
        Method solveIKMethod = MainFrame.class.getDeclaredMethod("solveIKForTrajectoryPoint", double.class, double.class, double.class, boolean.class);
        solveIKMethod.setAccessible(true);
        Method isWithinLimitsMethod = MainFrame.class.getDeclaredMethod("isWithinLimits", double[].class, boolean.class);
        isWithinLimitsMethod.setAccessible(true);

        // Khởi tạo IK cho điểm trước đó (giả lập điểm 30)
        double pt30_x = 38.0;
        double pt30_y = -22.5;
        double pt30_z = 100.0;
        
        double[] startQ = (double[]) solveIKMethod.invoke(frame, pt30_x, pt30_y, pt30_z, true);
        System.out.println("Start Q for Pt 30: " + (startQ != null ? "Found" : "Null"));
        
        if (startQ != null) {
            setField(frame, "trajectoryLastQ", startQ);
            
            for (int i = 0; i < targets.length; i++) {
                double[] pt = targets[i];
                System.out.printf("\n--- Processing Point %d: (X=%.2f, Y=%.2f, Z=%.2f) ---\n", 31 + i, pt[0], pt[1], pt[2]);
                
                double[] result = (double[]) solveIKMethod.invoke(frame, pt[0], pt[1], pt[2], true); // enforceJumpLimit=true
                
                if (result != null) {
                    System.out.println("[RESULT] Found direct solution on current branch.");
                    setField(frame, "trajectoryLastQ", result);
                } else {
                    System.out.println("[RESULT] Direct solution FAILED (or rejected due to huge jump). Branch transition search triggered...");
                    
                    double[] jumpResult = (double[]) solveIKMethod.invoke(frame, pt[0], pt[1], pt[2], false); // enforceJumpLimit=false
                    double[] trajLastQ = (double[]) getField(frame, "trajectoryLastQ");
                    
                    if (jumpResult != null && trajLastQ != null) {
                        double maxJointJump = 0.0;
                        for (int j = 0; j < 6; j++) {
                            double diff = Math.abs(jumpResult[j] - trajLastQ[j]); 
                            if (diff > maxJointJump) maxJointJump = diff;
                        }
                        
                        int N = (int) Math.ceil(maxJointJump / 15.0);
                        if (N < 1) N = 1;
                        System.out.printf("[TRANSITION] PHÁT HIỆN CẦN CHUYỂN NHÁNH! Max Jump = %.2f deg -> Chèn %d điểm phụ.\n", maxJointJump, N);
                        
                        setField(frame, "trajectoryLastQ", jumpResult);
                    } else {
                        System.out.println("[FAIL] Completely failed IK (no branch solutions).");
                    }
                }
            }
        }
        
        System.out.println("\n=== Test Completed Successfully ===");
        System.exit(0);
    }
    
    private static void setField(Object obj, String fieldName, Object value) throws Exception {
        Field f = obj.getClass().getDeclaredField(fieldName);
        f.setAccessible(true);
        f.set(obj, value);
    }
    private static Object getField(Object obj, String fieldName) throws Exception {
        Field f = obj.getClass().getDeclaredField(fieldName);
        f.setAccessible(true);
        return f.get(obj);
    }
}
