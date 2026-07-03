package kinematics;

import java.util.ArrayList;
import java.util.List;

/**
 * [ignoring loop detection]
 * Bộ lập kế hoạch quỹ đạo không gian khớp (Joint-Space Trajectory Planner) chuyên nghiệp.
 * Sử dụng nội suy đa thức bậc 5 (Quintic Spline) để đảm bảo độ mượt mà của chuyển động.
 * Hỗ trợ hai phương án điều kiện biên gia tốc:
 * - Phương án A: Ép gia tốc tại mọi điểm neo = 0 (đơn giản, dễ code nhưng có velocity ripple).
 * - Phương án B: Giải hệ tridiagonal (thuật toán Thomas) để đảm bảo liên tục C2 toàn cục.
 * Đồng thời, tích hợp bộ co giãn thời gian (retiming loop) đồng bộ giúp bảo toàn hình dáng quỹ đạo
 * và không vượt quá các giới hạn vật lý của động cơ (vận tốc góc, gia tốc góc).
 */
public class TrajectoryPlanner {

    private static final int NUM_JOINTS = 6;

    /**
     * Hàm điều phối chính lập kế hoạch quỹ đạo.
     * 
     * @param filledJoints Danh sách các điểm neo khớp đã được giải động học ngược và làm đầy.
     * @param maxVelocityDegPerSec Vận tốc khớp tối đa (độ/giây).
     * @param maxAccelerationDegPerSec2 Gia tốc khớp tối đa (độ/giây^2).
     * @param useGlobalC2 Có sử dụng tính liên tục C2 toàn cục qua giải hệ tridiagonal hay không.
     * @param dt Bước thời gian mẫu (ví dụ 0.030 giây ~ 30ms).
     * @return Danh sách các cấu hình khớp nội suy với tần số mẫu dt.
     */
    public static List<double[]> planTrajectory(
            List<double[]> filledJoints,
            double maxVelocityDegPerSec,
            double maxAccelerationDegPerSec2,
            boolean useGlobalC2,
            double dt
    ) {
        int numPoints = filledJoints.size();
        if (numPoints < 2) return filledJoints;

        int N = numPoints - 1; // số phân đoạn spline
        double[][] Q = new double[numPoints][6];
        for (int i = 0; i < numPoints; i++) {
            Q[i] = filledJoints.get(i).clone();
        }

        // Khởi tạo thời gian cho mỗi phân đoạn ban đầu bằng dt (30ms)
        double[] H = new double[N];
        for (int k = 0; k < N; k++) {
            H[k] = dt;
        }

        double[][] V = new double[numPoints][6];
        double[][] A = new double[numPoints][6];

        boolean violated = true;
        int iter = 0;
        int maxIter = 100; // Giới hạn số lần lặp tránh treo vô hạn

        // Vòng lặp co giãn thời gian (Retiming Loop) để bảo toàn vận tốc và gia tốc
        while (violated && iter < maxIter) {
            violated = false;
            iter++;

            // Bước 1: Tính toán vận tốc khớp trung gian vk bằng Siciliano 3-point
            computeViaVelocities(Q, H, V);

            // Bước 2: Tính toán gia tốc khớp trung gian ak
            if (useGlobalC2) {
                // Phương án B: Giải hệ phương trình tuyến tính băng để đảm bảo liên tục C2 toàn cục
                computeGlobalC2Accelerations(Q, H, V, A);
            } else {
                // Phương án A: Ép gia tốc bằng không tại tất cả các điểm neo
                for (int i = 0; i < numPoints; i++) {
                    for (int j = 0; j < NUM_JOINTS; j++) {
                        A[i][j] = 0.0;
                    }
                }
            }

            // Bước 3: Kiểm tra giới hạn vận tốc/gia tốc tại các điểm dọc phân đoạn
            double[] segScales = new double[N];
            for (int k = 0; k < N; k++) {
                segScales[k] = 1.0;

                // Quét 10 điểm trung gian trong phân đoạn k
                for (int m = 0; m <= 10; m++) {
                    double tau = H[k] * m / 10.0;
                    for (int j = 0; j < NUM_JOINTS; j++) {
                        double vel = evaluateSplineVelocity(Q[k][j], Q[k+1][j], V[k][j], V[k+1][j], A[k][j], A[k+1][j], H[k], tau);
                        double acc = evaluateSplineAcceleration(Q[k][j], Q[k+1][j], V[k][j], V[k+1][j], A[k][j], A[k+1][j], H[k], tau);

                        if (Math.abs(vel) > maxVelocityDegPerSec) {
                            double s = Math.abs(vel) / maxVelocityDegPerSec;
                            if (s > segScales[k]) segScales[k] = s;
                            violated = true;
                        }
                        if (Math.abs(acc) > maxAccelerationDegPerSec2) {
                            double s = Math.sqrt(Math.abs(acc) / maxAccelerationDegPerSec2);
                            if (s > segScales[k]) segScales[k] = s;
                            violated = true;
                        }
                    }
                }
            }

            if (violated) {
                // Giãn thời gian các phân đoạn bị vi phạm
                for (int k = 0; k < N; k++) {
                    if (segScales[k] > 1.0) {
                        H[k] *= segScales[k];
                    }
                }
            }
        }

        // Bước 4: Nội suy ra chuỗi quỹ đạo độ phân giải cao với bước dt (30ms)
        List<double[]> trajectory = new ArrayList<>();
        double totalTime = 0.0;
        for (int k = 0; k < N; k++) {
            totalTime += H[k];
        }

        double t = 0.0;
        int currentSegment = 0;
        double segmentStartTime = 0.0;

        while (t <= totalTime + 1e-9) {
            // Xác định phân đoạn hiện tại chứa t
            while (currentSegment < N - 1 && t > segmentStartTime + H[currentSegment] + 1e-9) {
                segmentStartTime += H[currentSegment];
                currentSegment++;
            }

            double tau = t - segmentStartTime;
            if (tau > H[currentSegment]) {
                tau = H[currentSegment];
            }

            double[] q_sample = new double[NUM_JOINTS];
            for (int j = 0; j < NUM_JOINTS; j++) {
                q_sample[j] = evaluateSplinePosition(
                        Q[currentSegment][j],
                        Q[currentSegment+1][j],
                        V[currentSegment][j],
                        V[currentSegment+1][j],
                        A[currentSegment][j],
                        A[currentSegment+1][j],
                        H[currentSegment],
                        tau
                );
                q_sample[j] = normalizeAngle(q_sample[j]);
            }
            trajectory.add(q_sample);
            t += dt;
        }

        return trajectory;
    }

    /**
     * Vá các lỗ hổng (các điểm null do IK thất bại) trong quỹ đạo thô.
     */
    public static List<double[]> fillGaps(List<double[]> rawJointTrajectory, double[] defaultAngles) {
        List<double[]> jointTrajectory = new ArrayList<>();
        int size = rawJointTrajectory.size();
        for (int i = 0; i < size; i++) {
            double[] rawQ = rawJointTrajectory.get(i);
            if (rawQ != null) {
                jointTrajectory.add(rawQ.clone());
            } else {
                int prevIdx = -1;
                for (int k = i - 1; k >= 0; k--) {
                    if (rawJointTrajectory.get(k) != null) {
                        prevIdx = k;
                        break;
                    }
                }

                int nextIdx = -1;
                for (int k = i + 1; k < size; k++) {
                    if (rawJointTrajectory.get(k) != null) {
                        nextIdx = k;
                        break;
                    }
                }

                if (prevIdx != -1 && nextIdx != -1) {
                    double[] q_prev = rawJointTrajectory.get(prevIdx);
                    double[] q_next = rawJointTrajectory.get(nextIdx);

                    // Kiểm tra tương thích cấu hình khớp: bước nhảy lớn > 30 độ
                    boolean compatible = true;
                    for (int j = 0; j < NUM_JOINTS; j++) {
                        if (Math.abs(wrappedDegDiff(q_next[j], q_prev[j])) > 30.0) {
                            compatible = false;
                            break;
                        }
                    }

                    if (compatible) {
                        // Nội suy đa thức bậc 3 cho các đoạn nhỏ hợp lệ
                        double[] q_interp = new double[NUM_JOINTS];
                        int N = nextIdx - prevIdx;
                        double t = (double) (i - prevIdx) / N;
                        double factor = 3.0 * t * t - 2.0 * t * t * t;

                        for (int j = 0; j < NUM_JOINTS; j++) {
                            double deltaQ = wrappedDegDiff(q_next[j], q_prev[j]);
                            double q_val = q_prev[j] + deltaQ * factor;
                            q_interp[j] = normalizeAngle(q_val);
                        }
                        jointTrajectory.add(q_interp);
                    } else {
                        // Giữ cấu hình hợp lệ gần nhất nếu không tương thích cấu hình
                        double[] fallbackQ = ((i - prevIdx) <= (nextIdx - i)) ? q_prev.clone() : q_next.clone();
                        jointTrajectory.add(fallbackQ);
                    }
                } else {
                    // Vướng biên: chọn cấu hình biên gần nhất
                    double[] fallbackQ;
                    if (prevIdx != -1) {
                        fallbackQ = rawJointTrajectory.get(prevIdx).clone();
                    } else if (nextIdx != -1) {
                        fallbackQ = rawJointTrajectory.get(nextIdx).clone();
                    } else {
                        fallbackQ = defaultAngles.clone();
                    }
                    jointTrajectory.add(fallbackQ);
                }
            }
        }
        return jointTrajectory;
    }

    /**
     * Tính toán vận tốc khớp trung gian vk bằng Siciliano 3-point.
     * Tự động ép v_k = 0 nếu khớp đổi chiều di chuyển để tránh dao động (overshoot).
     */
    private static void computeViaVelocities(double[][] Q, double[] H, double[][] V) {
        int numPoints = Q.length;
        int N = numPoints - 1;
        for (int j = 0; j < NUM_JOINTS; j++) {
            V[0][j] = 0.0;
            V[N][j] = 0.0;
            for (int k = 1; k < N; k++) {
                double diff_prev = wrappedDegDiff(Q[k][j], Q[k-1][j]);
                double diff_next = wrappedDegDiff(Q[k+1][j], Q[k][j]);
                double v_prev = diff_prev / H[k-1];
                double v_next = diff_next / H[k];

                double v = 0.5 * (v_prev + v_next);
                if (v_prev * v_next <= 0.0) {
                    v = 0.0;
                } else if (Math.abs(Math.abs(diff_prev) - Math.abs(diff_next)) > 5.0) {
                    v = 0.0; // Điểm gãy khúc hình học (kink) do chuyển tiếp nhánh Cartesian/Joint-space
                }
                V[k][j] = v;
            }
        }
    }

    /**
     * Tính toán gia tốc khớp trung gian ak bằng hệ phương trình tuyến tính tridiagonal (C2 Continuity).
     */
    private static void computeGlobalC2Accelerations(double[][] Q, double[] H, double[][] V, double[][] A) {
        int numPoints = Q.length;
        int N = numPoints - 1;
        int M = N - 1; // số ẩn số gia tốc a_1 ... a_{N-1}

        if (M <= 0) {
            for (int i = 0; i < numPoints; i++) {
                for (int j = 0; j < NUM_JOINTS; j++) {
                    A[i][j] = 0.0;
                }
            }
            return;
        }

        double[] c = new double[M];
        double[] d = new double[M];
        double[] e = new double[M];
        double[] r = new double[M];

        for (int j = 0; j < NUM_JOINTS; j++) {
            for (int i = 1; i < N; i++) {
                int idx = i - 1;
                double h_prev = H[i-1];
                double h_curr = H[i];

                c[idx] = -1.0 / h_prev;
                d[idx] = 3.0 * (1.0 / h_prev + 1.0 / h_curr);
                e[idx] = -1.0 / h_curr;

                double diff_prev = wrappedDegDiff(Q[i][j], Q[i-1][j]);
                double diff_next = wrappedDegDiff(Q[i+1][j], Q[i][j]);

                double term1 = (10.0 * diff_next - (6.0 * V[i][j] + 4.0 * V[i+1][j]) * h_curr) / (h_curr * h_curr * h_curr);
                double term2 = (10.0 * diff_prev - (4.0 * V[i-1][j] + 6.0 * V[i][j]) * h_prev) / (h_prev * h_prev * h_prev);

                r[idx] = 2.0 * (term1 - term2);
            }

            // Giải hệ phương trình tuyến tính tridiagonal sử dụng thuật toán Thomas
            double[] solution = solveThomas(c, d, e, r);
            A[0][j] = 0.0;
            A[N][j] = 0.0;
            if (solution != null) {
                for (int i = 1; i < N; i++) {
                    A[i][j] = solution[i - 1];
                }
            } else {
                for (int i = 1; i < N; i++) {
                    A[i][j] = 0.0;
                }
            }
        }
    }

    /**
     * Thuật toán Thomas (Tridiagonal Matrix Algorithm) để giải hệ phương trình tuyến tính ba đường chéo trong O(N).
     */
    public static double[] solveThomas(double[] c, double[] d, double[] e, double[] r) {
        int M = d.length;
        double[] cp = new double[M];
        double[] rp = new double[M];
        double[] x = new double[M];

        if (Math.abs(d[0]) < 1e-12) return null;
        cp[0] = e[0] / d[0];
        rp[0] = r[0] / d[0];

        for (int i = 1; i < M; i++) {
            double denom = d[i] - c[i] * cp[i - 1];
            if (Math.abs(denom) < 1e-12) return null;
            cp[i] = e[i] / denom;
            rp[i] = (r[i] - c[i] * rp[i - 1]) / denom;
        }

        x[M - 1] = rp[M - 1];
        for (int i = M - 2; i >= 0; i--) {
            x[i] = rp[i] - cp[i] * x[i + 1];
        }
        return x;
    }

    // Các hàm phụ trợ nội suy đa thức bậc 5 cho từng phân đoạn
    private static double evaluateSplinePosition(double q0, double q1, double v0, double v1, double a0, double a1, double h, double tau) {
        double D = wrappedDegDiff(q1, q0);
        double a3 = (10.0 * D - (6.0 * v0 + 4.0 * v1) * h - (1.5 * a0 - 0.5 * a1) * h * h) / (h * h * h);
        double a4 = (-15.0 * D + (8.0 * v0 + 7.0 * v1) * h + (1.5 * a0 - a1) * h * h) / (h * h * h * h);
        double a5 = (6.0 * D - 3.0 * (v0 + v1) * h - 0.5 * (a0 - a1) * h * h) / (h * h * h * h * h);

        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;
        double tau5 = tau4 * tau;

        return q0 + v0 * tau + 0.5 * a0 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5;
    }

    private static double evaluateSplineVelocity(double q0, double q1, double v0, double v1, double a0, double a1, double h, double tau) {
        double D = wrappedDegDiff(q1, q0);
        double a3 = (10.0 * D - (6.0 * v0 + 4.0 * v1) * h - (1.5 * a0 - 0.5 * a1) * h * h) / (h * h * h);
        double a4 = (-15.0 * D + (8.0 * v0 + 7.0 * v1) * h + (1.5 * a0 - a1) * h * h) / (h * h * h * h);
        double a5 = (6.0 * D - 3.0 * (v0 + v1) * h - 0.5 * (a0 - a1) * h * h) / (h * h * h * h * h);

        double tau2 = tau * tau;
        double tau3 = tau2 * tau;
        double tau4 = tau3 * tau;

        return v0 + a0 * tau + 3.0 * a3 * tau2 + 4.0 * a4 * tau3 + 5.0 * a5 * tau4;
    }

    private static double evaluateSplineAcceleration(double q0, double q1, double v0, double v1, double a0, double a1, double h, double tau) {
        double D = wrappedDegDiff(q1, q0);
        double a3 = (10.0 * D - (6.0 * v0 + 4.0 * v1) * h - (1.5 * a0 - 0.5 * a1) * h * h) / (h * h * h);
        double a4 = (-15.0 * D + (8.0 * v0 + 7.0 * v1) * h + (1.5 * a0 - a1) * h * h) / (h * h * h * h);
        double a5 = (6.0 * D - 3.0 * (v0 + v1) * h - 0.5 * (a0 - a1) * h * h) / (h * h * h * h * h);

        double tau2 = tau * tau;
        double tau3 = tau2 * tau;

        return a0 + 6.0 * a3 * tau + 12.0 * a4 * tau2 + 20.0 * a5 * tau3;
    }

    private static double wrappedDegDiff(double a, double b) {
        double d = a - b;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }

    private static double normalizeAngle(double angle) {
        double a = angle;
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }
}
