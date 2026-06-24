package scratch;

public class TestFK2 {
    public static void main(String[] args) {
        System.out.println("Right Arm q3=0:");
        for (double q4 = -140; q4 <= -30; q4 += 20) {
            double[] q = { 0, 0, 0, q4, 0, 0 };
            double[][] pts = TestFK.computeFK(q, true);
            System.out.printf("  q4=%+6.1f -> Shoulder:[%+.2f, %+.2f, %+.2f] Elbow:[%+.2f, %+.2f, %+.2f] Wrist:[%+.2f, %+.2f, %+.2f]\n",
                q4, pts[3][0], pts[3][1], pts[3][2],
                pts[4][0], pts[4][1], pts[4][2],
                pts[5][0], pts[5][1], pts[5][2]);
        }
    }
}
