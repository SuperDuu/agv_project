import json
import math
import os
import time
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AgvArmPlanner(Node):
    """ROS 2 planner node for the 6-DOF robotic arm.

    Subscribes to `/agv_arm/plan_requests` and publishes the computed joint
    trajectory to `/agv_arm/plan_responses`.
    """

    def __init__(self):
        super().__init__("agv_arm_planner")
        self.publisher = self.create_publisher(String, "/agv_arm/plan_responses", 10)
        self.subscription = self.create_subscription(
            String, "/agv_arm/plan_requests", self.handle_plan_request, 10
        )

        # Load robot parameters from YAML configuration file
        self.load_robot_geometry()
        self.get_logger().info("AGV Arm Planner started. Subscribing to /agv_arm/plan_requests")

    def load_robot_geometry(self):
        config_path = "/workspaces/agv_ros2/src/agv_arm_description/config/agv_arm_geometry.yaml"
        # Fallback to local workspace path if running on host
        if not os.path.exists(config_path):
            config_path = "src/agv_arm_description/config/agv_arm_geometry.yaml"

        try:
            with open(config_path, "r", encoding="utf-8") as handle:
                config = yaml.safe_load(handle)
            links = config["links"]
            self.L0 = float(links["L0"])
            self.L1 = float(links["L1"])
            self.L2 = float(links["L2"])
            self.L3 = float(links["L3"])
            self.L4 = float(links["L4"])
            self.L5 = float(links["L5"])
            self.L6 = float(links["L6"])
            self.L7 = float(links["L7"])
            self.get_logger().info(f"Loaded geometry from {config_path}")
        except Exception as exc:
            self.get_logger().warn(f"Failed to load yaml config ({exc}). Using prototype fallbacks.")
            self.L0 = 130.0
            self.L1 = 0.0
            self.L2 = 32.0
            self.L3 = 0.0
            self.L4 = 20.0
            self.L5 = 25.0
            self.L6 = 0.0
            self.L7 = 15.0

    def get_mdh_matrix(self, alpha, d, a, offset, q):
        theta = q + offset
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        return np.array([
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def get_tool_matrix(self):
        return np.array([
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, -self.L7],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def compute_fk(self, q, is_right):
        d2 = (self.L2 + self.L3) if is_right else -(self.L2 + self.L3)
        q6_kinematic = q[5] if is_right else -q[5]
        params = [
            (0.0, self.L1 + self.L0, 0.0, -math.pi / 2, q[0]),
            (-math.pi / 2, d2, 0.0, -math.pi / 2, q[1]),
            (-math.pi / 2, 0.0, 0.0, -math.pi, q[2]),
            (0.0, 0.0, self.L4, -math.pi / 2, q[3]),
            (-math.pi / 2, self.L5 + self.L6, 0.0, 0.0, q[4]),
            (-math.pi / 2, 0.0, 0.0, 0.0, q6_kinematic)
        ]
        T = np.eye(4)
        for alpha, d, a, offset, qi in params:
            T = T @ self.get_mdh_matrix(alpha, d, a, offset, qi)
        T = T @ self.get_tool_matrix()
        return T

    def compute_jacobian(self, q, is_right):
        d2 = (self.L2 + self.L3) if is_right else -(self.L2 + self.L3)
        q6_kinematic = q[5] if is_right else -q[5]
        params = [
            (0.0, self.L1 + self.L0, 0.0, -math.pi / 2, q[0]),
            (-math.pi / 2, d2, 0.0, -math.pi / 2, q[1]),
            (-math.pi / 2, 0.0, 0.0, -math.pi, q[2]),
            (0.0, 0.0, self.L4, -math.pi / 2, q[3]),
            (-math.pi / 2, self.L5 + self.L6, 0.0, 0.0, q[4]),
            (-math.pi / 2, 0.0, 0.0, 0.0, q6_kinematic)
        ]
        T = np.eye(4)
        z0 = []
        p0 = []
        for alpha, d, a, offset, qi in params:
            ca = math.cos(alpha)
            sa = math.sin(alpha)
            RxTx = np.array([
                [1.0, 0.0, 0.0, a],
                [0.0, ca, -sa, 0.0],
                [0.0, sa, ca, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
            T_i_prime = T @ RxTx
            joint_axis = T_i_prime[0:3, 2].copy()
            if not is_right and len(z0) == 5:
                joint_axis = -joint_axis
            z0.append(joint_axis)
            p0.append(T_i_prime[0:3, 3])

            theta = qi + offset
            ct = math.cos(theta)
            st = math.sin(theta)
            RzTz = np.array([
                [ct, -st, 0.0, 0.0],
                [st, ct, 0.0, 0.0],
                [0.0, 0.0, 1.0, d],
                [0.0, 0.0, 0.0, 1.0]
            ])
            T = T_i_prime @ RzTz

        T_tool = T @ self.get_tool_matrix()
        p_tool = T_tool[0:3, 3]
        R_tool = T_tool[0:3, 0:3]

        J0 = np.zeros((6, 6))
        for i in range(6):
            dp = p_tool - p0[i]
            J0[0:3, i] = np.cross(z0[i], dp)
            J0[3:6, i] = z0[i]

        RT = R_tool.T
        Je = np.zeros((6, 6))
        for j in range(6):
            Je[0:3, j] = RT @ J0[0:3, j]
            Je[3:6, j] = RT @ J0[3:6, j]
        return Je

    def orthonormalize_3x3(self, R):
        len0 = math.sqrt(R[0, 0]**2 + R[1, 0]**2 + R[2, 0]**2)
        if len0 < 1e-9:
            len0 = 1.0
        R[0, 0] /= len0
        R[1, 0] /= len0
        R[2, 0] /= len0
        
        dot = R[0, 0]*R[0, 1] + R[1, 0]*R[1, 1] + R[2, 0]*R[2, 1]
        R[0, 1] -= dot * R[0, 0]
        R[1, 1] -= dot * R[1, 0]
        R[2, 1] -= dot * R[2, 0]
        
        len1 = math.sqrt(R[0, 1]**2 + R[1, 1]**2 + R[2, 1]**2)
        if len1 < 1e-9:
            len1 = 1.0
        R[0, 1] /= len1
        R[1, 1] /= len1
        R[2, 1] /= len1
        
        R[0, 2] = R[1, 0]*R[2, 1] - R[2, 0]*R[1, 1]
        R[1, 2] = R[2, 0]*R[0, 1] - R[0, 0]*R[2, 1]
        R[2, 2] = R[0, 0]*R[1, 1] - R[1, 0]*R[0, 1]
        return R

    def solve_ik(self, px, py, pz, R_target, q_init_deg, is_right):
        q = np.array([math.radians(deg) for deg in q_init_deg])

        if is_right:
            min_lim = np.array([math.radians(deg) for deg in [-45, -90, 20, -95, -90, 0]])
            max_lim = np.array([math.radians(deg) for deg in [45, 90, 165, -15, 90, 90]])
        else:
            min_lim = np.array([math.radians(deg) for deg in [-45, -90, -165, 15, -90, 0]])
            max_lim = np.array([math.radians(deg) for deg in [45, 90, -20, 95, 90, 90]])

        T_target = np.eye(4)
        T_target[0:3, 0:3] = R_target
        T_target[0, 3] = px
        T_target[1, 3] = py
        T_target[2, 3] = pz

        max_iter = 100
        tol = 1e-4
        alpha = 0.8
        best_q = q.copy()
        best_err = float('inf')
        prev_err_norm = float('inf')

        for _ in range(max_iter):
            T_curr = self.compute_fk(q, is_right)
            dp = T_target[0:3, 3] - T_curr[0:3, 3]

            R_curr = T_curr[0:3, 0:3]
            R_rel = R_curr.T @ R_target
            R_rel = self.orthonormalize_3x3(R_rel)

            trace = np.trace(R_rel)
            cos_theta = 0.5 * (trace - 1.0)
            cos_theta = max(-1.0, min(1.0, cos_theta))
            theta = math.acos(cos_theta)

            dw = np.zeros(3)
            if theta >= 1e-6:
                if theta > 3.0:
                    dw[0] = theta * math.sqrt(max(0.0, 0.5 * (R_rel[0, 0] + 1.0)))
                    dw[1] = theta * math.sqrt(max(0.0, 0.5 * (R_rel[1, 1] + 1.0)))
                    dw[2] = theta * math.sqrt(max(0.0, 0.5 * (R_rel[2, 2] + 1.0)))
                    if R_rel[2, 1] - R_rel[1, 2] < 0: dw[0] = -dw[0]
                    if R_rel[0, 2] - R_rel[2, 0] < 0: dw[1] = -dw[1]
                    if R_rel[1, 0] - R_rel[0, 1] < 0: dw[2] = -dw[2]
                else:
                    s = 0.5 * theta / math.sin(theta)
                    dw[0] = (R_rel[2, 1] - R_rel[1, 2]) * s
                    dw[1] = (R_rel[0, 2] - R_rel[2, 0]) * s
                    dw[2] = (R_rel[1, 0] - R_rel[0, 1]) * s

            dp_local = R_curr.T @ dp
            delta = np.hstack((dp_local, dw))

            err_norm = np.linalg.norm(delta)
            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()

            if err_norm < tol:
                sol = [math.degrees(rad) for rad in q]
                if self.is_within_limits(sol, is_right, min_lim, max_lim):
                    return sol

            # Adaptive step size
            if err_norm > prev_err_norm:
                alpha *= 0.5
            else:
                alpha = min(0.95, alpha * 1.05)
            prev_err_norm = err_norm

            Je = self.compute_jacobian(q, is_right)
            lam = 0.05
            try:
                A = Je @ Je.T + (lam**2) * np.eye(6)
                dq = Je.T @ np.linalg.solve(A, delta)
            except Exception:
                dq = np.zeros(6)

            for i in range(6):
                next_q = q[i] + alpha * dq[i]
                next_q = (next_q + math.pi) % (2 * math.pi) - math.pi
                if next_q < min_lim[i] or next_q > max_lim[i]:
                    next_q = max(min_lim[i], min(max_lim[i], next_q))
                q[i] = next_q

        sol = [math.degrees(rad) for rad in best_q]
        if best_err < 0.50 and self.is_within_limits(sol, is_right, min_lim, max_lim):
            return sol
        return None

    def is_within_limits(self, joints, is_right, min_lim, max_lim):
        for i in range(6):
            rad = math.radians(joints[i])
            if rad < min_lim[i] - 0.01 or rad > max_lim[i] + 0.01:
                return False
        return True

    def solve_ik_smart(self, px, py, pz, current_joints, is_right, preferred_config="+", preferred_alpha=None, preferred_offset=None):
        alpha_scan = [0.0, -15.0, 15.0, -30.0, 30.0, -45.0, -60.0, -75.0, -90.0]
        if preferred_alpha is not None and preferred_alpha in alpha_scan:
            alpha_scan.remove(preferred_alpha)
            alpha_scan.insert(0, preferred_alpha)
            
        q1_min = -45.0
        q1_max = 45.0
        
        q1_base = math.atan2(py, px) if is_right else -math.atan2(py, -px)
        q1_base = max(math.radians(q1_min), min(math.radians(q1_max), q1_base))
        
        yaw_offsets = [0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -75.0, 75.0, -90.0, 90.0]
        if preferred_offset is not None and preferred_offset in yaw_offsets:
            yaw_offsets.remove(preferred_offset)
            yaw_offsets.insert(0, preferred_offset)
        
        # --- PASS 1: Search for preferred configuration ---
        for alpha in alpha_scan:
            alpha_rad = math.radians(alpha)
            ca = math.cos(math.pi + alpha_rad)
            sa = math.sin(math.pi + alpha_rad)
            R_y = np.array([
                [ca, 0.0, sa],
                [0.0, 1.0, 0.0],
                [-sa, 0.0, ca]
            ])
            
            for offset_deg in yaw_offsets:
                yaw = q1_base + math.radians(offset_deg)
                if is_right:
                    cy = math.cos(yaw)
                    sy = math.sin(yaw)
                    R_z = np.array([
                        [cy, -sy, 0.0],
                        [sy, cy, 0.0],
                        [0.0, 0.0, 1.0]
                    ])
                    R_target = R_z @ R_y
                else:
                    yawR = -yaw
                    cyR = math.cos(yawR)
                    syR = math.sin(yawR)
                    R_z_right = np.array([
                        [cyR, -syR, 0.0],
                        [syR, cyR, 0.0],
                        [0.0, 0.0, 1.0]
                    ])
                    R_target_right = R_z_right @ R_y
                    R_target = np.array([
                        [R_target_right[0, 0], -R_target_right[0, 1], -R_target_right[0, 2]],
                        [-R_target_right[1, 0], R_target_right[1, 1], R_target_right[1, 2]],
                        [-R_target_right[2, 0], R_target_right[2, 1], R_target_right[2, 2]]
                    ])
                
                sol = self.solve_ik(px, py, pz, R_target, current_joints, is_right)
                if sol is not None:
                    actual_cfg = "+"
                    if is_right:
                        actual_cfg = "+" if sol[2] >= 0 else "-"
                    else:
                        actual_cfg = "+" if sol[2] <= 0 else "-"
                    
                    if actual_cfg == preferred_config:
                        # Found a matching configuration! We can early exit immediately!
                        return sol, alpha, offset_deg

        # --- PASS 2 (FALLBACK): Search for any configuration ---
        best_sol = None
        best_cost = float('inf')
        best_alpha = None
        best_offset = None
        
        for alpha in alpha_scan:
            alpha_rad = math.radians(alpha)
            ca = math.cos(math.pi + alpha_rad)
            sa = math.sin(math.pi + alpha_rad)
            R_y = np.array([
                [ca, 0.0, sa],
                [0.0, 1.0, 0.0],
                [-sa, 0.0, ca]
            ])
            
            for offset_deg in yaw_offsets:
                yaw = q1_base + math.radians(offset_deg)
                if is_right:
                    cy = math.cos(yaw)
                    sy = math.sin(yaw)
                    R_z = np.array([
                        [cy, -sy, 0.0],
                        [sy, cy, 0.0],
                        [0.0, 0.0, 1.0]
                    ])
                    R_target = R_z @ R_y
                else:
                    yawR = -yaw
                    cyR = math.cos(yawR)
                    syR = math.sin(yawR)
                    R_z_right = np.array([
                        [cyR, -syR, 0.0],
                        [syR, cyR, 0.0],
                        [0.0, 0.0, 1.0]
                    ])
                    R_target_right = R_z_right @ R_y
                    R_target = np.array([
                        [R_target_right[0, 0], -R_target_right[0, 1], -R_target_right[0, 2]],
                        [-R_target_right[1, 0], R_target_right[1, 1], R_target_right[1, 2]],
                        [-R_target_right[2, 0], R_target_right[2, 1], R_target_right[2, 2]]
                    ])
                
                sol = self.solve_ik(px, py, pz, R_target, current_joints, is_right)
                if sol is not None:
                    jump_penalty = sum((sol[i] - current_joints[i])**2 for i in range(6))
                    if jump_penalty < best_cost:
                        best_cost = jump_penalty
                        best_sol = sol
                        best_alpha = alpha
                        best_offset = offset_deg
                        
        if best_sol is not None:
            return best_sol, best_alpha, best_offset
            
        return None, None, None

    def handle_plan_request(self, msg):
        try:
            request = json.loads(msg.data)
            request_id = request.get("request_id")
            arm = request.get("arm", "right")
            is_right = arm == "right"
            req_type = request.get("type", "plan_pose")
            current_joints = request.get("current_joints", [0.0] * 6)
            preferred_config = request.get("preferred_config", "+")

            if req_type == "plan_path":
                path_pts = request.get("path", [])
                self.get_logger().info(f"Received path request {request_id} for {arm} arm with {len(path_pts)} points")
                
                trajectory = []
                current_q = list(current_joints)
                ok = True
                error_msg = ""
                
                pref_alpha = None
                pref_offset = None
                
                for idx, pt in enumerate(path_pts):
                    px = float(pt.get("x", 0.0))
                    py = float(pt.get("y", 0.0))
                    pz = float(pt.get("z", 0.0))
                    
                    solved_joints, alpha, offset = self.solve_ik_smart(px, py, pz, current_q, is_right, preferred_config, pref_alpha, pref_offset)
                    
                    if solved_joints is None:
                        ok = False
                        error_msg = f"IK solver failed at point {idx} ({px}, {py}, {pz})"
                        break
                    
                    pref_alpha = alpha
                    pref_offset = offset
                    
                    # Interpolate from previous position to solved position
                    # Limit joint speed to max_joint_vel = 30 deg/sec.
                    # With DT = 0.03s, max change per step is 30.0 * 0.03 = 0.9 degrees.
                    max_diff = max(abs(solved_joints[i] - current_q[i]) for i in range(6))
                    steps = max(5, int(math.ceil(max_diff / 0.9)))
                    for step in range(1, steps + 1):
                        t = float(step) / steps
                        q_step = [
                            current_q[i] + (solved_joints[i] - current_q[i]) * t
                            for i in range(6)
                        ]
                        trajectory.append(q_step)
                    
                    current_q = solved_joints

                if not ok:
                    self.get_logger().warn(f"Path planning failed: {error_msg}")
                    response = {
                        "type": "plan_response",
                        "request_id": request_id,
                        "ok": False,
                        "error": error_msg,
                        "stamp": time.time()
                    }
                else:
                    self.get_logger().info(f"Path planned successfully with {len(trajectory)} steps")
                    response = {
                        "type": "plan_response",
                        "request_id": request_id,
                        "ok": True,
                        "status": "planned",
                        "trajectory": trajectory,
                        "stamp": time.time()
                    }
            else:
                target = request.get("target", {})
                px = float(target.get("x", 0.0))
                py = float(target.get("y", 0.0))
                pz = float(target.get("z", 0.0))

                self.get_logger().info(f"Received request {request_id} for {arm} arm to ({px}, {py}, {pz})")

                # Solve Inverse Kinematics
                solved_joints, _, _ = self.solve_ik_smart(px, py, pz, current_joints, is_right, preferred_config)

                if solved_joints is None:
                    self.get_logger().warn(f"IK solver failed for ({px}, {py}, {pz})")
                    response = {
                        "type": "plan_response",
                        "request_id": request_id,
                        "ok": False,
                        "error": "IK solver failed to converge or target out of workspace",
                        "stamp": time.time()
                    }
                else:
                    self.get_logger().info(f"IK solved successfully: {solved_joints}")
                    # Generate joint-space trajectory by interpolating
                    # Limit joint speed to max_joint_vel = 30 deg/sec.
                    # With DT = 0.03s, max change per step is 30.0 * 0.03 = 0.9 degrees.
                    max_diff = max(abs(solved_joints[i] - current_joints[i]) for i in range(6))
                    steps = max(25, int(math.ceil(max_diff / 0.9)))
                    trajectory = []
                    for step in range(steps + 1):
                        t = float(step) / steps
                        q_step = [
                            current_joints[i] + (solved_joints[i] - current_joints[i]) * t
                            for i in range(6)
                        ]
                        trajectory.append(q_step)

                    response = {
                        "type": "plan_response",
                        "request_id": request_id,
                        "ok": True,
                        "status": "planned",
                        "trajectory": trajectory,
                        "stamp": time.time()
                    }

            # Publish response
            resp_msg = String()
            resp_msg.data = json.dumps(response)
            self.publisher.publish(resp_msg)

        except Exception as exc:
            self.get_logger().error(f"Error processing plan request: {exc}")


def main():
    rclpy.init()
    node = AgvArmPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
