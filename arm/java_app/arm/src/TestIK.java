import kinematics.Kinematics;
import static kinematics.Kinematics.*;

public class TestIK {
    public static void main(String[] args) {
        double sx = 7.22, sy = 0.39, sz = 10.24;
        double ex = -8.00, ey = 8.00, ez = 15.00;
        double alphaDeg = -45.0; // Let's test what happens if it's -45
        
        System.out.println("Testing solveIK trajectory math with alpha = -45:");
        
        double[] currentAngles = {0, 0, 0, 0, 0}; 
        
        for(double r = 0; r <= 1.0; r += 0.1) {
            double tx = sx + (ex - sx) * r;
            double ty = sy + (ey - sy) * r;
            double tz = sz + (ez - sz) * r;
            
            double[] q = solveIKContinuous(tx, ty, tz, alphaDeg, currentAngles);
            
            if (q == null) {
                System.out.printf("r=%.1f : solveIK returned NULL for target %.2f, %.2f, %.2f\n", r, tx, ty, tz);
            } else {
                currentAngles = q;
                double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
                double[][] params = {
                        { 0, L1, 0, 0, Math.toRadians(q[0]) },
                        { -Math.PI / 2, L2, 0, -Math.PI / 2, Math.toRadians(q[1]) },
                        { 0, -L4, L3, Math.PI / 2, Math.toRadians(q[2]) },
                        { Math.PI / 2, L5, 0, Math.PI / 2, Math.toRadians(q[3]) },
                        { -Math.PI / 2, 0, 0, 0, Math.toRadians(q[4]) }
                };
                for (int i = 0; i < NUM_JOINTS; i++) {
                    T = multiply4x4(T, getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
                }
                T = multiply4x4(T, getToolMatrix());
                double err = Math.sqrt(Math.pow(tx - T[0][3], 2) + Math.pow(ty - T[1][3], 2) + Math.pow(tz - T[2][3], 2));
                System.out.printf("r=%.1f : Err: %.4f | Pos: %.2f, %.2f, %.2f\n", r, err, T[0][3], T[1][3], T[2][3]);
            }
        }
    }
    
    public static double[] solveIKContinuous(double px, double py, double pz, double alphaDeg, double[] referenceAngles) {
        String[] configs = { "+", "-" };
        double[] bestQ = null;
        double minJointDistSq = Double.MAX_VALUE;

        for (String c1 : configs) {
            double[][] dynamicR = getAchievableOrientation(px, py, pz, alphaDeg, c1);
            for (String c2 : configs) {
                double[] q = Kinematics.solveIK(px, py, pz, dynamicR, c1, c2);
                if (q != null) {
                    double distSq = 0;
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        double d = q[i] - referenceAngles[i];
                        while (d > 180) d -= 360;
                        while (d < -180) d += 360;
                        distSq += d * d;
                    }
                    if (distSq < minJointDistSq) {
                        minJointDistSq = distSq;
                        bestQ = q;
                    }
                }
            }
        }
        return bestQ;
    }

    public static double[][] getAchievableOrientation(double px, double py, double pz, double alphaDeg, String cfg) {
        double r_xy = Math.sqrt(px * px + py * py);
        double A_off = L4 - L2;
        double phi = Math.atan2(py, px);
        double delta = Math.atan2(A_off, Math.sqrt(Math.max(0, r_xy * r_xy - A_off * A_off)));
        double q1 = cfg.equals("+") ? (phi + delta) : (phi + Math.PI - delta);

        double alpha_rad = Math.toRadians(alphaDeg);
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        double c1 = Math.cos(q1), s1 = Math.sin(q1);
        double[][] R_z = { { c1, -s1, 0 }, { s1, c1, 0 }, { 0, 0, 1 } };
        return multiplyMatrices(R_z, R_y);
    }
}
