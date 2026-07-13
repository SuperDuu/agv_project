import kinematics.Kinematics;

public class FindSmallestX {
    public static void main(String[] args) {
        double bestX = Double.MAX_VALUE;
        double[] bestQ = null;
        double[] bestP = null;
        double bestPitch = 0;

        for (double q3 = 20.0; q3 <= 165.0; q3 += 0.5) {
            for (double sum = -46.0; sum <= -26.0; sum += 0.5) {
                double q4 = sum - (-90.0) - q3;
                double[] q = { 0.0, -90.0, q3, q4, 0.0, 30.0 };
                if (isWithinLimits(q)) {
                    double[] p = computeFK(q);
                    if (p[0] < bestX) {
                        bestX = p[0];
                        bestQ = q.clone();
                        bestP = p.clone();
                        bestPitch = sum;
                    }
                }
            }
        }

        if (bestQ != null) {
            System.out.printf("Best retracted configuration at q2 = -90.0:\n");
            System.out.printf("q=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                    bestQ[0], bestQ[1], bestQ[2], bestQ[3], bestQ[4], bestQ[5]);
            System.out.printf("TCP=(%.2f, %.2f, %.2f) | pitch=%.2f\n",
                    bestP[0], bestP[1], bestP[2], bestPitch);
        } else {
            System.out.println("No valid configurations found!");
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
