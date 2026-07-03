import kinematics.Kinematics;

public class DebugIK {
    public static void main(String[] args) {
        double px = 33.79;
        double py = 28.44;
        double pz = 100.00;
        
        // Let's construct R_target from tryAlpha
        double alphaDeg = -66.0; // let's try a few alphas
        double alpha_rad = Math.toRadians(alphaDeg);
        double ca = Math.cos(Math.PI + alpha_rad);
        double sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        
        double q1_base = Math.atan2(py, px);
        double offsetDeg = -78.9;
        double yaw = q1_base + Math.toRadians(offsetDeg);
        double cy = Math.cos(yaw);
        double sy = Math.sin(yaw);
        double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
        double[][] R_target = kinematics.Kinematics.multiplyMatrices(R_z, R_y);
        
        double[] qInit = {
            Math.toRadians(0.0),
            Math.toRadians(0.0),
            Math.toRadians(10.0),
            Math.toRadians(-30.0),
            Math.toRadians(0.0),
            Math.toRadians(0.0)
        };
        
        System.out.println("Running detailed solveIK...");
        double[] q = solveIKDetailed(px, py, pz, R_target, qInit, true);
    }
    
    public static double[] solveIKDetailed(double px, double py, double pz, double[][] R_target, double[] qInitRad, boolean isRight) {
        double[] q = qInitRad.clone();
        int maxIter = 100;
        double tol = 1e-4;
        double alpha = 0.5;
        double bestErrNorm = Double.MAX_VALUE;
        double[] bestQ = q.clone();
        double prevErrNorm = Double.MAX_VALUE;

        double[][] T_target = {
                { R_target[0][0], R_target[0][1], R_target[0][2], px },
                { R_target[1][0], R_target[1][1], R_target[1][2], py },
                { R_target[2][0], R_target[2][1], R_target[2][2], pz },
                { 0, 0, 0, 1 }
        };

        double[] minLimRad = new double[6];
        double[] maxLimRad = new double[6];
        double[] JOINT_MIN_RIGHT = { -45, -90, -90, -140, -90, -90 };
        double[] JOINT_MAX_RIGHT = { 45, 90, 90, -30, 90, 90 };
        
        for (int i = 0; i < 6; i++) {
            minLimRad[i] = Math.toRadians(JOINT_MIN_RIGHT[i]);
            maxLimRad[i] = Math.toRadians(JOINT_MAX_RIGHT[i]);
        }

        for (int iter = 0; iter < maxIter; iter++) {
            double[][] T_curr = kinematics.Kinematics.computeFKMatrix(q, isRight);
            double[] e = kinematics.Kinematics.computeTr2Delta(T_curr, T_target);

            double errNorm = 0;
            for (int i = 0; i < 6; i++) {
                errNorm += e[i] * e[i];
            }
            errNorm = Math.sqrt(errNorm);

            if (errNorm < bestErrNorm) {
                bestErrNorm = errNorm;
                bestQ = q.clone();
            }

            if (errNorm > prevErrNorm) {
                alpha *= 0.5;
            } else {
                alpha = Math.min(0.95, alpha * 1.05);
            }
            prevErrNorm = errNorm;

            double[][] J = kinematics.Kinematics.computeJacobianEE(q, isRight);
            double detJ = kinematics.Kinematics.compute6x6Determinant(J);
            double manipulability = Math.abs(detJ);
            double lambda = 0.01;
            if (manipulability < 0.008) {
                double ratio = manipulability / 0.008;
                lambda = Math.sqrt(0.01*0.01 + 0.4*0.4 * (1 - ratio)*(1 - ratio));
            }

            double[] dq = kinematics.Kinematics.solveDLS(J, e, lambda);
            
            System.out.printf("Iter %3d | errNorm=%.4f | alpha=%.4f | manipulability=%.6f | lambda=%.4f\n",
                iter, errNorm, alpha, manipulability, lambda);
            System.out.printf("  Error vector: dx=%.4f, dy=%.4f, dz=%.4f, dwx=%.4f, dwy=%.4f, dwz=%.4f\n",
                e[0], e[1], e[2], e[3], e[4], e[5]);
            System.out.printf("  q (deg) : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                Math.toDegrees(q[0]), Math.toDegrees(q[1]), Math.toDegrees(q[2]),
                Math.toDegrees(q[3]), Math.toDegrees(q[4]), Math.toDegrees(q[5]));
            System.out.printf("  dq(deg) : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                Math.toDegrees(dq[0]), Math.toDegrees(dq[1]), Math.toDegrees(dq[2]),
                Math.toDegrees(dq[3]), Math.toDegrees(dq[4]), Math.toDegrees(dq[5]));

            if (errNorm < tol) {
                System.out.println("CONVERGED!");
                break;
            }

            for (int i = 0; i < 6; i++) {
                double nextQ = kinematics.Kinematics.wrapToPi(q[i] + alpha * dq[i]);
                if (nextQ < minLimRad[i] || nextQ > maxLimRad[i]) {
                    dq[i] = 0;
                    nextQ = Math.max(minLimRad[i], Math.min(maxLimRad[i], nextQ));
                }
                q[i] = nextQ;
            }
        }

        double[][] T_best = kinematics.Kinematics.computeFKMatrix(bestQ, isRight);
        double dx = T_best[0][3] - px;
        double dy = T_best[1][3] - py;
        double dz = T_best[2][3] - pz;
        double posErr = Math.sqrt(dx*dx + dy*dy + dz*dz);
        System.out.printf("Final Best posErr=%.4f cm (%.4f mm)\n", posErr, posErr * 10.0);
        
        return null;
    }
}
