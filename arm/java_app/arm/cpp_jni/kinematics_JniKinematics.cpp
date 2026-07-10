#define _USE_MATH_DEFINES
#include <jni.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include "kinematics_JniKinematics.h"

// Robot dimensions (must match Kinematics.java)
const double L0 = 130.0;
const double L1 = 0.0;
const double L2 = 32.0;
const double L3 = 0.0;
const double L4 = 20.0;
const double L5 = 25.0;
const double L6 = 0.0;
const double L7 = 15.0;

// θ-space limits (must match Kinematics.java JOINT_MIN/MAX)
const double JOINT_MIN_RIGHT[6] = { -45, -90, 20, -95, -90, -90 };
const double JOINT_MAX_RIGHT[6] = { 45, 90, 165, -15, 90, 90 };
const double JOINT_MIN_LEFT[6]  = { -45, -90, -165, 15, -90, -90 };
const double JOINT_MAX_LEFT[6]  = { 45, 90, -20, 95, 90, 90 };

// Check θ-space limits
// Mirrors Kinematics.java isWithinLimits() exactly
bool isWithinLimits(const double thetaDeg[6], bool isRight) {
    const double* jMin = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
    const double* jMax = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
    // θ-space limits
    for (int i = 0; i < 6; i++) {
        if (thetaDeg[i] < jMin[i] - 0.1 || thetaDeg[i] > jMax[i] + 0.1) return false;
    }
    return true;
}

// Helper matrix functions
void multiply4x4(const double A[4][4], const double B[4][4], double C[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void getMDHMatrix(double alpha, double d, double a, double offset, double q, double out[4][4]) {
    double theta = q + offset;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    out[0][0] = ct;      out[0][1] = -st;     out[0][2] = 0;   out[0][3] = a;
    out[1][0] = st * ca; out[1][1] = ct * ca; out[1][2] = -sa;  out[1][3] = -sa * d;
    out[2][0] = st * sa; out[2][1] = ct * sa; out[2][2] = ca;   out[2][3] = ca * d;
    out[3][0] = 0;       out[3][1] = 0;       out[3][2] = 0;   out[3][3] = 1;
}

void getToolMatrix(double out[4][4]) {
    out[0][0] = 0; out[0][1] = -1; out[0][2] = 0;  out[0][3] = 0;
    out[1][0] = 0; out[1][1] = 0;  out[1][2] = -1; out[1][3] = -L7;
    out[2][0] = 1; out[2][1] = 0;  out[2][2] = 0;  out[2][3] = 0;
    out[3][0] = 0; out[3][1] = 0;  out[3][2] = 0;  out[3][3] = 1;
}

void computeFKMatrix(const double q[6], bool isRight, double T[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            T[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    double d2 = isRight ? (L2 + L3) : -(L2 + L3);
    double params[6][5] = {
        { 0, L1 + L0, 0, -M_PI / 2, q[0] },
        { -M_PI / 2, d2, 0, -M_PI / 2, q[1] },
        { -M_PI / 2, 0, 0, -M_PI, q[2] },
        { 0, 0, L4, -M_PI / 2, q[3] },
        { -M_PI / 2, L5 + L6, 0, 0, q[4] },
        { -M_PI / 2, 0, 0, 0, q[5] }
    };
    for (int i = 0; i < 6; i++) {
        double MDH[4][4], T_next[4][4];
        getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4], MDH);
        multiply4x4(T, MDH, T_next);
        for (int r = 0; r < 4; r++) std::copy(T_next[r], T_next[r] + 4, T[r]);
    }
    double MDH[4][4], T_next[4][4];
    getToolMatrix(MDH);
    multiply4x4(T, MDH, T_next);
    for (int r = 0; r < 4; r++) std::copy(T_next[r], T_next[r] + 4, T[r]);
}

void orthonormalize3x3(const double R[3][3], double Q[3][3]) {
    double len0 = std::sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0] + R[2][0]*R[2][0]);
    if (len0 < 1e-9) len0 = 1.0;
    Q[0][0] = R[0][0] / len0;
    Q[1][0] = R[1][0] / len0;
    Q[2][0] = R[2][0] / len0;
    
    double dot = Q[0][0]*R[0][1] + Q[1][0]*R[1][1] + Q[2][0]*R[2][1];
    double v1_x = R[0][1] - dot * Q[0][0];
    double v1_y = R[1][1] - dot * Q[1][0];
    double v1_z = R[2][1] - dot * Q[2][0];
    double len1 = std::sqrt(v1_x*v1_x + v1_y*v1_y + v1_z*v1_z);
    if (len1 < 1e-9) len1 = 1.0;
    Q[0][1] = v1_x / len1;
    Q[1][1] = v1_y / len1;
    Q[2][1] = v1_z / len1;
    
    Q[0][2] = Q[1][0]*Q[2][1] - Q[2][0]*Q[1][1];
    Q[1][2] = Q[2][0]*Q[0][1] - Q[0][0]*Q[2][1];
    Q[2][2] = Q[0][0]*Q[1][1] - Q[1][0]*Q[0][1];
}

double compute6x6Determinant(const double A[6][6]) {
    double M[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) M[i][j] = A[i][j];
    }
    double det = 1.0;
    int swaps = 0;
    for (int i = 0; i < 6; i++) {
        int pivotRow = i;
        for (int r = i + 1; r < 6; r++) {
            if (std::abs(M[r][i]) > std::abs(M[pivotRow][i])) {
                pivotRow = r;
            }
        }
        if (pivotRow != i) {
            for (int c = 0; c < 6; c++) std::swap(M[i][c], M[pivotRow][c]);
            swaps++;
        }
        if (std::abs(M[i][i]) < 1e-12) return 0.0;
        det *= M[i][i];
        for (int r = i + 1; r < 6; r++) {
            double factor = M[r][i] / M[i][i];
            for (int c = i; c < 6; c++) {
                M[r][c] -= factor * M[i][c];
            }
        }
    }
    if (swaps % 2 == 1) det = -det;
    return det;
}

bool solveLinearSystem(const double A[6][6], const double b[6], double x[6]) {
    double M[6][7];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) M[i][j] = A[i][j];
        M[i][6] = b[i];
    }
    for (int i = 0; i < 6; i++) {
        int max_r = i;
        for (int k = i + 1; k < 6; k++) {
            if (std::abs(M[k][i]) > std::abs(M[max_r][i])) max_r = k;
        }
        if (max_r != i) {
            for (int c = 0; c < 7; c++) std::swap(M[i][c], M[max_r][c]);
        }
        if (std::abs(M[i][i]) < 1e-12) return false;
        for (int k = i + 1; k < 6; k++) {
            double factor = M[k][i] / M[i][i];
            for (int j = i; j <= 6; j++) {
                M[k][j] -= factor * M[i][j];
            }
        }
    }
    for (int i = 5; i >= 0; i--) {
        double sum = 0;
        for (int j = i + 1; j < 6; j++) {
            sum += M[i][j] * x[j];
        }
        x[i] = (M[i][6] - sum) / M[i][i];
    }
    return true;
}

void computeJacobianEE(const double q[6], bool isRight, double Je[6][6]) {
    double T[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) T[i][j] = (i == j) ? 1.0 : 0.0;
    }
    double d2 = isRight ? (L2 + L3) : -(L2 + L3);
    double params[6][5] = {
        { 0, L1 + L0, 0, -M_PI / 2, q[0] },
        { -M_PI / 2, d2, 0, -M_PI / 2, q[1] },
        { -M_PI / 2, 0, 0, -M_PI, q[2] },
        { 0, 0, L4, -M_PI / 2, q[3] },
        { -M_PI / 2, L5 + L6, 0, 0, q[4] },
        { -M_PI / 2, 0, 0, 0, q[5] }
    };
    double z0[6][3];
    double p0[6][3];
    for (int i = 0; i < 6; i++) {
        double alpha = params[i][0];
        double a = params[i][2];
        double d = params[i][1];
        double offset = params[i][3];
        double qi = params[i][4];

        double ca = std::cos(alpha), sa = std::sin(alpha);
        double RxTx[4][4] = {
            { 1, 0, 0, a },
            { 0, ca, -sa, 0 },
            { 0, sa, ca, 0 },
            { 0, 0, 0, 1 }
        };
        double T_i_prime[4][4];
        multiply4x4(T, RxTx, T_i_prime);

        z0[i][0] = T_i_prime[0][2];
        z0[i][1] = T_i_prime[1][2];
        z0[i][2] = T_i_prime[2][2];

        p0[i][0] = T_i_prime[0][3];
        p0[i][1] = T_i_prime[1][3];
        p0[i][2] = T_i_prime[2][3];

        double theta = qi + offset;
        double ct = std::cos(theta), st = std::sin(theta);
        double RzTz[4][4] = {
            { ct, -st, 0, 0 },
            { st, ct, 0, 0 },
            { 0, 0, 1, d },
            { 0, 0, 0, 1 }
        };
        multiply4x4(T_i_prime, RzTz, T);
    }
    double T_tool[4][4];
    double MDH_tool[4][4];
    getToolMatrix(MDH_tool);
    multiply4x4(T, MDH_tool, T_tool);

    double p_tool_x = T_tool[0][3];
    double p_tool_y = T_tool[1][3];
    double p_tool_z = T_tool[2][3];

    double R_tool[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) R_tool[i][j] = T_tool[i][j];
    }

    double J0[6][6];
    for (int i = 0; i < 6; i++) {
        double dx = p_tool_x - p0[i][0];
        double dy = p_tool_y - p0[i][1];
        double dz = p_tool_z - p0[i][2];

        J0[0][i] = z0[i][1] * dz - z0[i][2] * dy;
        J0[1][i] = z0[i][2] * dx - z0[i][0] * dz;
        J0[2][i] = z0[i][0] * dy - z0[i][1] * dx;

        J0[3][i] = z0[i][0];
        J0[4][i] = z0[i][1];
        J0[5][i] = z0[i][2];
    }

    double RT[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) RT[i][j] = R_tool[j][i];
    }

    for (int j = 0; j < 6; j++) {
        Je[0][j] = RT[0][0] * J0[0][j] + RT[0][1] * J0[1][j] + RT[0][2] * J0[2][j];
        Je[1][j] = RT[1][0] * J0[0][j] + RT[1][1] * J0[1][j] + RT[1][2] * J0[2][j];
        Je[2][j] = RT[2][0] * J0[0][j] + RT[2][1] * J0[1][j] + RT[2][2] * J0[2][j];

        Je[3][j] = RT[0][0] * J0[3][j] + RT[0][1] * J0[4][j] + RT[0][2] * J0[5][j];
        Je[4][j] = RT[1][0] * J0[3][j] + RT[1][1] * J0[4][j] + RT[1][2] * J0[5][j];
        Je[5][j] = RT[2][0] * J0[3][j] + RT[2][1] * J0[4][j] + RT[2][2] * J0[5][j];
    }
}

void solveDLS(const double J[6][6], const double e[6], double lambda, double dq[6]) {
    double JJT[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            double sum = 0;
            for (int k = 0; k < 6; k++) sum += J[i][k] * J[j][k];
            JJT[i][j] = sum;
        }
        JJT[i][i] += lambda * lambda;
    }
    double x[6];
    bool success = solveLinearSystem(JJT, e, x);
    if (!success) {
        for (int i = 0; i < 6; i++) dq[i] = 0;
        return;
    }
    for (int i = 0; i < 6; i++) {
        double sum = 0;
        for (int k = 0; k < 6; k++) sum += J[k][i] * x[k];
        dq[i] = sum;
    }
}

double wrapToPi(double rad) {
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

// Full DLS solver in C++
bool solveIK_CppDLS(double px, double py, double pz, const double R_target[3][3], const double qInitRad[6], bool isRight, double qOut[6], int maxIter = 200) {
    std::copy(qInitRad, qInitRad + 6, qOut);
    double bestQ[6];
    std::copy(qInitRad, qInitRad + 6, bestQ);
    double bestErrNorm = 1e30;

    double T_target[4][4] = {
        { R_target[0][0], R_target[0][1], R_target[0][2], px },
        { R_target[1][0], R_target[1][1], R_target[1][2], py },
        { R_target[2][0], R_target[2][1], R_target[2][2], pz },
        { 0, 0, 0, 1 }
    };

    double minLimRad[6], maxLimRad[6];
    for (int i = 0; i < 6; i++) {
        minLimRad[i] = (isRight ? JOINT_MIN_RIGHT[i] : JOINT_MIN_LEFT[i]) * M_PI / 180.0;
        maxLimRad[i] = (isRight ? JOINT_MAX_RIGHT[i] : JOINT_MAX_LEFT[i]) * M_PI / 180.0;
    }

    double alpha = 0.8;
    double prevErrNorm = 1e30;
    double tol = 1e-5;

    for (int iter = 0; iter < maxIter; iter++) {
        double T_curr[4][4];
        computeFKMatrix(qOut, isRight, T_curr);

        double R0[3][3], R1[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R0[i][j] = T_curr[i][j];
                R1[i][j] = T_target[i][j];
            }
        }
        double R0T[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) R0T[i][j] = R0[j][i];
        }
        double R_rel[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R_rel[i][j] = 0;
                for (int k = 0; k < 3; k++) R_rel[i][j] += R0T[i][k] * R1[k][j];
            }
        }

        // Orthonormalize relative rotation matrix to prevent numerical drift (Matches Java exactly!)
        double R_rel_ortho[3][3];
        orthonormalize3x3(R_rel, R_rel_ortho);

        double dx = T_target[0][3] - T_curr[0][3];
        double dy = T_target[1][3] - T_curr[1][3];
        double dz = T_target[2][3] - T_curr[2][3];

        double dp[3] = {
            R0T[0][0]*dx + R0T[0][1]*dy + R0T[0][2]*dz,
            R0T[1][0]*dx + R0T[1][1]*dy + R0T[1][2]*dz,
            R0T[2][0]*dx + R0T[2][1]*dy + R0T[2][2]*dz
        };

        double trace = R_rel_ortho[0][0] + R_rel_ortho[1][1] + R_rel_ortho[2][2];
        double cosTheta = 0.5 * (trace - 1.0);
        cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
        double theta = std::acos(cosTheta);

        double dw[3] = {0, 0, 0};
        if (theta >= 1e-6) {
            if (theta > 3.0) {
                dw[0] = theta * std::sqrt(std::max(0.0, 0.5 * (R_rel_ortho[0][0] + 1.0)));
                dw[1] = theta * std::sqrt(std::max(0.0, 0.5 * (R_rel_ortho[1][1] + 1.0)));
                dw[2] = theta * std::sqrt(std::max(0.0, 0.5 * (R_rel_ortho[2][2] + 1.0)));
                if (R_rel_ortho[2][1] - R_rel_ortho[1][2] < 0) dw[0] = -dw[0];
                if (R_rel_ortho[0][2] - R_rel_ortho[2][0] < 0) dw[1] = -dw[1];
                if (R_rel_ortho[1][0] - R_rel_ortho[0][1] < 0) dw[2] = -dw[2];
            } else {
                double sinTheta = std::sin(theta);
                double s = (std::abs(sinTheta) > 1e-4) ? (0.5 * theta / sinTheta) : 0.5;
                dw[0] = (R_rel_ortho[2][1] - R_rel_ortho[1][2]) * s;
                dw[1] = (R_rel_ortho[0][2] - R_rel_ortho[2][0]) * s;
                dw[2] = (R_rel_ortho[1][0] - R_rel_ortho[0][1]) * s;
            }
        }

        double delta[6] = { dp[0], dp[1], dp[2], dw[0], dw[1], dw[2] };
        double errNorm = 0;
        for (int i = 0; i < 6; i++) errNorm += delta[i] * delta[i];
        errNorm = std::sqrt(errNorm);

        if (errNorm < bestErrNorm) {
            bestErrNorm = errNorm;
            std::copy(qOut, qOut + 6, bestQ);
        }

        if (errNorm > prevErrNorm) alpha *= 0.5;
        else alpha = std::min(0.95, alpha * 1.05);
        prevErrNorm = errNorm;

        if (errNorm < tol) return true;

        double Je[6][6];
        computeJacobianEE(qOut, isRight, Je);

        double detJ = compute6x6Determinant(Je);
        double manipulability = std::abs(detJ);
        double lambda = 0.01;
        if (manipulability < 0.008) {
            double ratio = manipulability / 0.008;
            lambda = std::sqrt(0.01*0.01 + 0.4*0.4 * (1 - ratio)*(1 - ratio));
        }

        double dq[6];
        solveDLS(Je, delta, lambda, dq);

        // Clamping with locking to force joint limits contribution (Matches Java exactly!)
        for (int i = 0; i < 6; i++) {
            double nextQ = wrapToPi(qOut[i] + alpha * dq[i]);
            if (nextQ < minLimRad[i] || nextQ > maxLimRad[i]) {
                dq[i] = 0.0; // Lock this joint!
                nextQ = std::max(minLimRad[i], std::min(maxLimRad[i], nextQ));
            }
            qOut[i] = nextQ;
        }
    }

    double T_best[4][4];
    computeFKMatrix(bestQ, isRight, T_best);
    double finalPosErr = std::sqrt(std::pow(T_best[0][3] - px, 2) + std::pow(T_best[1][3] - py, 2) + std::pow(T_best[2][3] - pz, 2));
    if (finalPosErr <= 0.30) {
        // Convert to degrees for limit check
        double candidateDeg[6];
        for (int i = 0; i < 6; i++) {
            double d = bestQ[i] * 180.0 / M_PI;
            while (d > 180.0) d -= 360.0;
            while (d < -180.0) d += 360.0;
            candidateDeg[i] = d;
        }
        if (!isWithinLimits(candidateDeg, isRight)) return false;
        std::copy(bestQ, bestQ + 6, qOut);
        return true;
    }
    return false;
}

// C++ IKFast (Analytical geometric approximation + 3 DLS loops)
bool solveIK_IKFast(double px, double py, double pz, const double R_target[3][3], const double qInitRad[6], bool isRight, double qOut[6]) {
    // 1. Calculate simplified wrist center
    double xw = px - R_target[0][2] * L7;
    double yw = py - R_target[1][2] * L7;
    double zw = pz - R_target[2][2] * L7;

    // 2. Analytical solve for base angle q0
    double q0 = std::atan2(yw, xw);
    
    // Project wrist to plane
    double r = std::sqrt(xw * xw + yw * yw);
    double h = zw - (L0 + L1);
    
    // Joint 2 and 3 geometry
    double link1 = isRight ? (L2 + L3) : -(L2 + L3);
    double link2 = L4;
    
    double d_sq = r * r + h * h;
    
    // Cosine rule
    double cos_q2 = (d_sq - link1 * link1 - link2 * link2) / (2.0 * std::abs(link1) * link2);
    cos_q2 = std::max(-1.0, std::min(1.0, cos_q2));
    
    double q2 = std::acos(cos_q2);
    if (qInitRad[2] < 0) q2 = -q2;
    
    double q1 = std::atan2(h, r) - std::atan2(link2 * std::sin(q2), std::abs(link1) + link2 * std::cos(q2));
    
    double q_guess[6] = { q0, q1, q2, qInitRad[3], qInitRad[4], qInitRad[5] };
    
    // 3. Fast DLS refinement (exactly 4 loops)
    return solveIK_CppDLS(px, py, pz, R_target, q_guess, isRight, qOut, 4);
}

JNIEXPORT jdoubleArray JNICALL Java_kinematics_JniKinematics_solveIKNative(
    JNIEnv *env, jclass clazz, 
    jdouble px, jdouble py, jdouble pz, 
    jobjectArray jR_target, jdoubleArray jqInitRad, jboolean isRight, jint mode
) {
    // 1. Extract rotation matrix
    double R_target[3][3];
    for (int i = 0; i < 3; i++) {
        jdoubleArray row = (jdoubleArray)env->GetObjectArrayElement(jR_target, i);
        jdouble* rdata = env->GetDoubleArrayElements(row, NULL);
        for (int j = 0; j < 3; j++) {
            R_target[i][j] = rdata[j];
        }
        env->ReleaseDoubleArrayElements(row, rdata, JNI_ABORT);
        env->DeleteLocalRef(row);
    }

    // 2. Extract initial joint angles
    jdouble* q_init = env->GetDoubleArrayElements(jqInitRad, NULL);
    double q_init_cpp[6];
    for (int i = 0; i < 6; i++) {
        q_init_cpp[i] = q_init[i];
    }
    env->ReleaseDoubleArrayElements(jqInitRad, q_init, JNI_ABORT);

    // 3. Solve IK based on mode
    double q_out[6];
    bool success = false;
    
    if (mode == 2) {
        success = solveIK_IKFast(px, py, pz, R_target, q_init_cpp, isRight, q_out);
    } else {
        success = solveIK_CppDLS(px, py, pz, R_target, q_init_cpp, isRight, q_out, 100);
    }

    if (!success) {
        return NULL;
    }

    // Convert rad back to degrees
    jdoubleArray result = env->NewDoubleArray(6);
    jdouble result_data[6];
    for (int i = 0; i < 6; i++) {
        double deg = q_out[i] * 180.0 / M_PI;
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        result_data[i] = deg;
    }
    env->SetDoubleArrayRegion(result, 0, 6, result_data);

    return result;
}
