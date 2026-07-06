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
            self.L0 = 120.0
            self.L1 = 5.0
            self.L2 = 10.0
            self.L3 = 10.0
            self.L4 = 20.0
            self.L5 = 20.0
            self.L6 = 10.0
            self.L7 = 10.0

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
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, -self.L7],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def compute_fk(self, q, is_right):
        d2 = (self.L2 + self.L3) if is_right else -(self.L2 + self.L3)
        params = [
            (0.0, self.L1 + self.L0, 0.0, -math.pi / 2, q[0]),
            (-math.pi / 2, d2, 0.0, -math.pi / 2, q[1]),
            (-math.pi / 2, 0.0, 0.0, -math.pi, q[2]),
            (0.0, 0.0, self.L4, -math.pi / 2, q[3]),
            (-math.pi / 2, self.L5 + self.L6, 0.0, -math.pi / 2, q[4]),
            (-math.pi / 2, 0.0, 0.0, 0.0, q[5])
        ]
        T = np.eye(4)
        for alpha, d, a, offset, qi in params:
            T = T @ self.get_mdh_matrix(alpha, d, a, offset, qi)
        T = T @ self.get_tool_matrix()
        return T

    def compute_jacobian(self, q, is_right):
        d2 = (self.L2 + self.L3) if is_right else -(self.L2 + self.L3)
        params = [
            (0.0, self.L1 + self.L0, 0.0, -math.pi / 2, q[0]),
            (-math.pi / 2, d2, 0.0, -math.pi / 2, q[1]),
            (-math.pi / 2, 0.0, 0.0, -math.pi, q[2]),
            (0.0, 0.0, self.L4, -math.pi / 2, q[3]),
            (-math.pi / 2, self.L5 + self.L6, 0.0, -math.pi / 2, q[4]),
            (-math.pi / 2, 0.0, 0.0, 0.0, q[5])
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
            z0.append(T_i_prime[0:3, 2])
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

    def solve_ik(self, px, py, pz, R_target, q_init_deg, is_right):
        q = np.array([math.radians(deg) for deg in q_init_deg])

        if is_right:
            min_lim = np.array([math.radians(deg) for deg in [-45, -90, -90, -140, -90, -90]])
            max_lim = np.array([math.radians(deg) for deg in [45, 90, 90, -30, 90, 90]])
        else:
            min_lim = np.array([math.radians(deg) for deg in [-45, -90, -90, 30, -90, -90]])
            max_lim = np.array([math.radians(deg) for deg in [45, 90, 90, 140, 90, 90]])

        T_target = np.eye(4)
        T_target[0:3, 0:3] = R_target
        T_target[0, 3] = px
        T_target[1, 3] = py
        T_target[2, 3] = pz

        max_iter = 200
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
            # Orthonormalize relative rotation
            try:
                U, _, Vt = np.linalg.svd(R_rel)
                R_rel = U @ Vt
            except Exception:
                pass

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
                inv_term = np.linalg.inv(Je @ Je.T + (lam**2) * np.eye(6))
                dq = Je.T @ inv_term @ delta
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

    def solve_ik_smart(self, px, py, pz, current_joints, is_right):
        alpha_scan = [0.0, -15.0, 15.0, -30.0, 30.0, -45.0, -60.0, -75.0, -90.0]
        
        q1_min = -45.0
        q1_max = 45.0
        
        q1_base = math.atan2(py, px) if is_right else -math.atan2(py, -px)
        q1_base = max(math.radians(q1_min), min(math.radians(q1_max), q1_base))
        
        yaw_offsets = [0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -75.0, 75.0, -90.0, 90.0]
        
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
                    return sol
        return None

    def handle_plan_request(self, msg):
        try:
            request = json.loads(msg.data)
            request_id = request.get("request_id")
            arm = request.get("arm", "right")
            is_right = arm == "right"
            req_type = request.get("type", "plan_pose")
            current_joints = request.get("current_joints", [0.0] * 6)

            if req_type == "plan_path":
                path_pts = request.get("path", [])
                self.get_logger().info(f"Received path request {request_id} for {arm} arm with {len(path_pts)} points")
                
                trajectory = []
                current_q = list(current_joints)
                ok = True
                error_msg = ""
                
                for idx, pt in enumerate(path_pts):
                    px = float(pt.get("x", 0.0))
                    py = float(pt.get("y", 0.0))
                    pz = float(pt.get("z", 0.0))
                    
                    solved_joints = self.solve_ik_smart(px, py, pz, current_q, is_right)
                    
                    if solved_joints is None:
                        ok = False
                        error_msg = f"IK solver failed at point {idx} ({px}, {py}, {pz})"
                        break
                    
                    # Interpolate from previous position to solved position
                    steps = 5
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
                solved_joints = self.solve_ik_smart(px, py, pz, current_joints, is_right)

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
                    trajectory = []
                    steps = 25
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
