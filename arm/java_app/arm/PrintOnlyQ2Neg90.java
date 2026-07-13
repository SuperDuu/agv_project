import kinematics.Kinematics;

public class PrintOnlyQ2Neg90 {
    public static void main(String[] args) {
        System.out.println("Valid configurations for q2 = -90.0:");
        for (double q3 = 20.0; q3 <= 160.0; q3 += 5.0) {
            for (double sum = -50.0; sum <= -20.0; sum += 2.0) {
                double q4 = sum - (-90.0) - q3;
                double[] q = { 0.0, -90.0, q3, q4, 0.0, 30.0 };
                if (isWithinLimits(q)) {
                    double[] p = computeFK(q);
                    System.out.printf("q3=%.1f q4=%.1f -> TCP=(%.2f, %.2f, %.2f) | pitch=%.1f\n",
                            q3, q4, p[0], p[1], p[2], sum);
                }
            }
        }
    }

    private static boolean isWithinLimits(double[] q) {
        double[] minLim = { -45, -90, 20, -95, -90, -60 };
        double[] maxLim = { 45, 90, 165, -15, 90, 60 };
        for (int i = 0; i < q.length; i++) {
            if (q[i] < (minLim[i] - 0.1) || q[i] > (maxLim[i] + 0.1))
                return false;
        }
        return true;
    }

    private static double[] computeFK(double[] q) {
        double[] qRad = new double[6];
        for (int i = 0; i < 6; i++) qRad[i] = Math.toRadians(q[i]);
        double[][] T = Kinematics.computeFKMatrix(qRad, true);
        return new double[] { T[0][3], T[1][3], T[2][3] };
    }
}
