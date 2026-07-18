"""Kinematic MuJoCo reach-and-avoid environment for the AGV dual arm.

Targets are sampled from random valid joint configurations, so every object is
inside the reachable workspace by construction. Obstacles are placed between
the shoulder and target, and collision is checked analytically against arm
segments to keep the reward independent from MuJoCo contact tuning.
"""

from __future__ import annotations

import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GEOMETRY = ROOT / "ros2_ws/src/agv_arm_description/config/agv_arm_geometry.yaml"
XML_CACHE = Path("/tmp/mjcf-export/agv_reach")


def _mm(value: float) -> float:
    return float(value) / 1000.0


def _deg(value: float) -> float:
    return math.radians(float(value))


def _fmt(values) -> str:
    return " ".join(f"{float(v):.8g}" for v in values)


def _tx(d: float) -> np.ndarray:
    out = np.eye(4)
    out[0, 3] = d
    return out


def _tz(d: float) -> np.ndarray:
    out = np.eye(4)
    out[2, 3] = d
    return out


def _rx(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array(
        [[1.0, 0.0, 0.0, 0.0], [0.0, ca, -sa, 0.0], [0.0, sa, ca, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )


def _rz(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array(
        [[ca, -sa, 0.0, 0.0], [sa, ca, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )


def _quat_from_matrix(R: np.ndarray) -> np.ndarray:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, R[:3, :3].reshape(-1))
    return quat


def _pose_attrs(T: np.ndarray) -> dict[str, str]:
    return {"pos": _fmt(T[:3, 3]), "quat": _fmt(_quat_from_matrix(T))}


def _segment_sphere_clearance(a: np.ndarray, b: np.ndarray, center: np.ndarray, radius: float) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return float(np.linalg.norm(center - a) - radius)
    t = float(np.clip(np.dot(center - a, ab) / denom, 0.0, 1.0))
    closest = a + t * ab
    return float(np.linalg.norm(center - closest) - radius)


def _segment_segment_distance(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> float:
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(p1 - p2))
    if a <= eps:
        s = 0.0
        t = float(np.clip(f / e, 0.0, 1.0))
    else:
        c = float(np.dot(d1, r))
        if e <= eps:
            t = 0.0
            s = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            s = float(np.clip((b * f - c * e) / denom, 0.0, 1.0)) if denom != 0.0 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t = 1.0
                s = float(np.clip((b - c) / a, 0.0, 1.0))
    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return float(np.linalg.norm(c1 - c2))


@dataclass
class ReachInfo:
    distance: float
    min_clearance: float
    parallel_score: float
    coordination_score: float
    success: bool
    collision: bool
    shielded: bool = False


class AgvReachEnv:
    """Dual-arm reaching task with obstacle avoidance and mirrored coordination."""

    def __init__(
        self,
        geometry_path: str | Path = DEFAULT_GEOMETRY,
        seed: int | None = None,
        max_steps: int = 160,
        obstacle_count: int = 2,
        render: bool = False,
        domain_randomization: bool = True,
        safety_shield: bool = True,
    ):
        self.geometry_path = Path(geometry_path)
        self.rng = np.random.default_rng(seed)
        self.max_steps = int(max_steps)
        self.obstacle_count = int(obstacle_count)
        self.render_enabled = render
        self.domain_randomization = bool(domain_randomization)
        self.safety_shield_enabled = bool(safety_shield)
        self.viewer = None
        self.dt = 0.05
        self.action_scale = np.deg2rad(6.0)
        self.target_radius = 0.06
        self.nominal_clearance_margin = 0.045
        self.clearance_margin = self.nominal_clearance_margin
        self.curriculum_level = 1.0

        self.config = self._load_config()
        self.arms = ("right", "left")
        self._lengths_cache = self._read_lengths()
        self._q_lower_cache = self._read_joint_limit_array("lower", as_angle=True)
        self._q_upper_cache = self._read_joint_limit_array("upper", as_angle=True)
        self._q_velocity_limits_cache = self._read_joint_limit_array("velocity", default=1.5, as_angle=False)
        self._q_mid_cache = 0.5 * (self._q_lower_cache + self._q_upper_cache)
        self._q_range_cache = np.maximum(self._q_upper_cache - self._q_lower_cache, 1e-6)
        self._ctrl_lower_cache = self._q_lower_cache.copy()
        self._ctrl_upper_cache = self._q_upper_cache.copy()
        for arm in self.arms:
            start = self._arm_index(arm) * 6
            self._ctrl_lower_cache[start + 3] = self._q_lower_cache[start + 2] + self._q_lower_cache[start + 3]
            self._ctrl_upper_cache[start + 3] = self._q_upper_cache[start + 2] + self._q_upper_cache[start + 3]
        self._joint_axis_signs_cache = {arm: self._compute_joint_axis_signs(arm) for arm in self.arms}
        self._joint_origins_cache = {arm: self._compute_joint_origins(arm) for arm in self.arms}
        self._tool_matrix_cache = self._compute_tool_matrix()
        self.xml_path = self._build_xml()
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.joint_names = [f"{arm}_joint_{i}" for arm in self.arms for i in range(1, 7)]
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.joint_names]
        self.qpos_addr = np.array([self.model.jnt_qposadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.qvel_addr = np.array([self.model.jnt_dofadr[jid] for jid in self.joint_ids], dtype=np.int32)
        self.target_geoms = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"target_{arm}") for arm in self.arms
        ]
        self.obstacle_geoms = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle_{i}") for i in range(self.obstacle_count)
        ]

        self.q = np.zeros(12, dtype=np.float64)
        self.prev_q = self.q.copy()
        self.prev_delta = np.zeros(12, dtype=np.float64)
        self.last_shielded = False
        self.shield_corrections = 0
        self.total_shield_corrections = 0
        self.targets = np.zeros((2, 3), dtype=np.float64)
        self.target_q = np.zeros(12, dtype=np.float64)
        self.obstacles = np.zeros((self.obstacle_count, 4), dtype=np.float64)
        self.steps = 0
        self.action_gain = np.ones(12, dtype=np.float64)
        self.action_noise_std = 0.0
        self.action_latency_steps = 0
        self.action_queue: list[np.ndarray] = []
        self.obs_q_noise_std = 0.0
        self.obs_target_noise_std = 0.0
        self.obs_obstacle_noise_std = 0.0

        self.obs_dim = 45 + 7 * self.obstacle_count
        self.action_dim = 12
        self.obs_mean = np.zeros(self.obs_dim, dtype=np.float32)
        self.obs_std = np.ones(self.obs_dim, dtype=np.float32)

    def _load_config(self) -> dict:
        with open(self.geometry_path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    def _read_joint_limit_array(self, field: str, default: float | None = None, as_angle: bool = False) -> np.ndarray:
        values = []
        for arm in self.arms:
            limits = self.config["joint_limits"][arm]
            for i in range(1, 7):
                value = limits[f"joint_{i}"].get(field, default)
                if value is None:
                    raise KeyError(f"Missing joint limit field: {arm}.joint_{i}.{field}")
                values.append(_deg(value) if as_angle else float(value))
        return np.array(values, dtype=np.float64)

    @property
    def q_lower(self) -> np.ndarray:
        return self._q_lower_cache

    @property
    def q_upper(self) -> np.ndarray:
        return self._q_upper_cache

    @property
    def q_mid(self) -> np.ndarray:
        return self._q_mid_cache

    @property
    def q_range(self) -> np.ndarray:
        return self._q_range_cache

    @property
    def q_velocity_limits(self) -> np.ndarray:
        return self._q_velocity_limits_cache

    @property
    def ctrl_lower(self) -> np.ndarray:
        return self._ctrl_lower_cache

    @property
    def ctrl_upper(self) -> np.ndarray:
        return self._ctrl_upper_cache

    def _read_lengths(self) -> dict[str, float]:
        links = self.config["links"]
        return {name: _mm(links[name]) for name in ["L0", "L1", "L2", "L3", "L4", "L5", "L6", "L7"]}

    def _lengths(self) -> dict[str, float]:
        return self._lengths_cache

    def _arm_index(self, side: str) -> int:
        return 0 if side == "right" else 1

    def _arm_slice(self, side: str) -> slice:
        start = self._arm_index(side) * 6
        return slice(start, start + 6)

    def _mirror_left_from_right(self, q_right: np.ndarray) -> np.ndarray:
        return np.array([q_right[0], q_right[1], -q_right[2], -q_right[3], q_right[4], q_right[5]], dtype=np.float64)

    def set_curriculum(self, level: float) -> None:
        self.curriculum_level = float(np.clip(level, 0.0, 1.0))

    def _sample_domain(self) -> None:
        level = self.curriculum_level
        if not self.domain_randomization:
            self.action_gain[:] = 1.0
            self.action_noise_std = 0.0
            self.action_latency_steps = 0
            self.obs_q_noise_std = 0.0
            self.obs_target_noise_std = 0.0
            self.obs_obstacle_noise_std = 0.0
            self.clearance_margin = self.nominal_clearance_margin
            self.action_queue = []
            return

        self.action_gain = self.rng.uniform(1.0 - 0.12 * level, 1.0 + 0.12 * level, size=12)
        self.action_noise_std = np.deg2rad(0.12 + 0.45 * level)
        self.action_latency_steps = int(self.rng.integers(0, 1 + max(1, int(round(2 * level)))))
        self.obs_q_noise_std = np.deg2rad(0.15 + 0.85 * level)
        self.obs_target_noise_std = 0.002 + 0.012 * level
        self.obs_obstacle_noise_std = 0.002 + 0.015 * level
        self.clearance_margin = self.nominal_clearance_margin * (0.8 + 0.45 * level)
        self.action_queue = [np.zeros(12, dtype=np.float64) for _ in range(self.action_latency_steps)]

    def _apply_action_domain(self, action: np.ndarray) -> np.ndarray:
        applied = np.asarray(action, dtype=np.float64).copy()
        if self.domain_randomization:
            if self.action_latency_steps > 0:
                self.action_queue.append(applied)
                applied = self.action_queue.pop(0)
            applied = applied * self.action_gain
            applied += self.rng.normal(0.0, self.action_noise_std / self.action_scale, size=12)
        return np.clip(applied, -1.0, 1.0)

    def _q_to_ctrl(self, q: np.ndarray) -> np.ndarray:
        ctrl = np.asarray(q, dtype=np.float64).copy()
        for arm in self.arms:
            start = self._arm_index(arm) * 6
            ctrl[start + 3] = ctrl[start + 2] + ctrl[start + 3]
        return ctrl

    def _ctrl_to_q(self, ctrl: np.ndarray) -> np.ndarray:
        q = np.asarray(ctrl, dtype=np.float64).copy()
        for arm in self.arms:
            start = self._arm_index(arm) * 6
            q[start + 3] = q[start + 3] - q[start + 2]
        return q

    def _enforce_ctrl_coupling(self, ctrl: np.ndarray, q1_source: str = "right") -> np.ndarray:
        ctrl = np.clip(np.asarray(ctrl, dtype=np.float64).copy(), self.ctrl_lower, self.ctrl_upper)
        right_q1 = self._arm_slice("right").start
        left_q1 = self._arm_slice("left").start
        if q1_source == "left":
            shared_q1 = ctrl[left_q1]
        elif q1_source == "average":
            shared_q1 = 0.5 * (ctrl[right_q1] + ctrl[left_q1])
        else:
            shared_q1 = ctrl[right_q1]
        shared_q1 = np.clip(
            shared_q1,
            max(self.ctrl_lower[right_q1], self.ctrl_lower[left_q1]),
            min(self.ctrl_upper[right_q1], self.ctrl_upper[left_q1]),
        )
        ctrl[right_q1] = shared_q1
        ctrl[left_q1] = shared_q1
        for arm in self.arms:
            start = self._arm_index(arm) * 6
            q3 = ctrl[start + 2]
            q4_min = q3 + self.q_lower[start + 3]
            q4_max = q3 + self.q_upper[start + 3]
            ctrl[start + 3] = np.clip(ctrl[start + 3], q4_min, q4_max)
        return ctrl

    def _enforce_q_coupling(self, q: np.ndarray, q1_source: str = "right") -> np.ndarray:
        q = np.clip(np.asarray(q, dtype=np.float64).copy(), self.q_lower, self.q_upper)
        return self._ctrl_to_q(self._enforce_ctrl_coupling(self._q_to_ctrl(q), q1_source=q1_source))

    def _point_box_clearance(self, point: np.ndarray, half_x: float, half_y: float, z_min: float, z_max: float) -> float:
        x, y, z = point
        dx = max(abs(float(x)) - half_x, 0.0)
        dy = max(abs(float(y)) - half_y, 0.0)
        dz = max(z_min - float(z), float(z) - z_max, 0.0)
        outside = math.sqrt(dx * dx + dy * dy + dz * dz)
        if outside > 0.0:
            return outside
        return -min(half_x - abs(float(x)), half_y - abs(float(y)), float(z) - z_min, z_max - float(z))

    def _torso_link_clearances(self, points: list[np.ndarray]) -> list[float]:
        visual = self.config["visual"]
        dscale = float(visual.get("decoration_scale", 1.0))
        half = _mm(0.5 * float(28.0) * dscale)
        link_radius = _mm(visual.get("link_radius", 60.0))
        L = self._lengths()
        clearances = []
        for a, b in zip(points[2:-1], points[3:]):
            for alpha in np.linspace(0.0, 1.0, 5):
                p = (1.0 - alpha) * a + alpha * b
                clearances.append(self._point_box_clearance(p, half, half, 0.0, L["L0"]) - link_radius)
        return clearances

    def _is_safe_q(self, q: np.ndarray) -> bool:
        info = self._reach_info(q)
        return info.min_clearance >= self.clearance_margin

    def _safety_project(self, q_start: np.ndarray, q_goal: np.ndarray) -> tuple[np.ndarray, bool]:
        q_goal = self._enforce_q_coupling(q_goal, q1_source="average")
        if not self.safety_shield_enabled or self._is_safe_q(q_goal):
            return q_goal, False
        for alpha in (0.75, 0.5, 0.35, 0.25, 0.15, 0.08, 0.04):
            q_trial = self._enforce_q_coupling(q_start + alpha * (q_goal - q_start), q1_source="average")
            if self._is_safe_q(q_trial):
                return q_trial, True
        return q_start.copy(), True

    def _numeric_tool_jacobian(self, side: str, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        base_tool = self.fk(q)[side][1]
        J = np.zeros((3, 12), dtype=np.float64)
        for j in range(12):
            q_plus = q.copy()
            q_plus[j] += eps
            q_plus = self._enforce_q_coupling(q_plus, q1_source="average")
            J[:, j] = (self.fk(q_plus)[side][1] - base_tool) / eps
        return J

    def teacher_action(self) -> np.ndarray:
        q = self.q.copy()
        if np.all(np.isfinite(self.target_q)):
            bounded_dq = np.clip(self.target_q - q, -self.action_scale, self.action_scale)
            q_goal, _ = self._safety_project(q, q + bounded_dq)
            safe_dq = q_goal - q
            if np.linalg.norm(safe_dq) > 1e-8:
                return np.clip(safe_dq / self.action_scale, -1.0, 1.0)
        return np.zeros(12, dtype=np.float64)

    def _compute_joint_axis_signs(self, side: str) -> np.ndarray:
        signs = np.array([1.0, 1.0, -1.0, -1.0, 1.0, -1.0], dtype=np.float64)
        if side == "left":
            signs[5] = 1.0
        return signs

    def _joint_axis_signs(self, side: str) -> np.ndarray:
        return self._joint_axis_signs_cache[side]

    def _compute_joint_origins(self, side: str) -> list[np.ndarray]:
        L = self._lengths()
        is_right = side == "right"
        d2 = -(L["L2"] + L["L3"]) if is_right else (L["L2"] + L["L3"])
        home_q3 = _deg(-20.0 if is_right else 20.0)
        home_q4 = _deg(35.0 if is_right else -35.0)
        return [
            _rz(-math.pi / 2.0) @ _tz(L["L1"] + L["L0"]),
            _rx(-math.pi / 2.0) @ _rz(-math.pi / 2.0) @ _tz(d2),
            _rx(-math.pi / 2.0) @ _rz(-math.pi + home_q3),
            _tx(L["L4"]) @ _rz(-math.pi / 2.0 + home_q4),
            _rx(-math.pi / 2.0) @ _tz(L["L5"] + L["L6"]),
            _rx(-math.pi / 2.0),
        ]

    def _joint_origins(self, side: str) -> list[np.ndarray]:
        return self._joint_origins_cache[side]

    def _compute_tool_matrix(self) -> np.ndarray:
        L = self._lengths()
        return np.array(
            [
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, -L["L7"]],
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def _tool_matrix(self) -> np.ndarray:
        return self._tool_matrix_cache

    def fk_arm(self, side: str, q_arm: np.ndarray) -> tuple[list[np.ndarray], np.ndarray, np.ndarray]:
        T = np.eye(4)
        points = [T[:3, 3].copy()]
        for origin, axis_sign, angle in zip(self._joint_origins(side), self._joint_axis_signs(side), q_arm):
            T = T @ origin
            points.append(T[:3, 3].copy())
            T = T @ _rz(float(axis_sign * angle))
        T_tool = T @ self._tool_matrix()
        points.append(T_tool[:3, 3].copy())
        return points, T_tool[:3, 3].copy(), T_tool[:3, :3].copy()

    def fk(self, q: np.ndarray) -> dict[str, tuple[list[np.ndarray], np.ndarray, np.ndarray]]:
        return {arm: self.fk_arm(arm, q[self._arm_slice(arm)]) for arm in self.arms}

    def _build_xml(self) -> Path:
        XML_CACHE.mkdir(parents=True, exist_ok=True)
        xml_path = XML_CACHE / f"agv_reach_{os.getpid()}.xml"
        L = self._lengths()
        joint_radius = _mm(self.config["visual"].get("joint_radius", 80.0))
        link_radius = _mm(self.config["visual"].get("link_radius", 60.0))
        dscale = float(self.config.get("visual", {}).get("decoration_scale", 1.0))
        dmm = lambda value: _mm(float(value) * dscale)

        mj = ET.Element("mujoco", {"model": "agv_reach"})
        ET.SubElement(mj, "compiler", {"angle": "radian", "coordinate": "local"})
        ET.SubElement(mj, "option", {"timestep": "0.005", "gravity": "0 0 0", "integrator": "implicitfast"})
        asset = ET.SubElement(mj, "asset")
        ET.SubElement(asset, "material", {"name": "right_arm", "rgba": "0.2 0.45 0.9 1"})
        ET.SubElement(asset, "material", {"name": "left_arm", "rgba": "0.9 0.15 0.12 1"})
        ET.SubElement(asset, "material", {"name": "link", "rgba": "0.55 0.56 0.56 1"})
        ET.SubElement(asset, "material", {"name": "base_mat", "rgba": "0.18 0.19 0.2 1"})
        ET.SubElement(asset, "material", {"name": "torso_mat", "rgba": "0.12 0.13 0.14 1"})
        ET.SubElement(asset, "material", {"name": "gripper_mat", "rgba": "0.9 0.9 0.85 1"})
        ET.SubElement(asset, "material", {"name": "target_right_mat", "rgba": "0.1 0.8 0.2 0.8"})
        ET.SubElement(asset, "material", {"name": "target_left_mat", "rgba": "1.0 0.78 0.1 0.85"})
        ET.SubElement(asset, "material", {"name": "obstacle_mat", "rgba": "0.9 0.15 0.1 0.65"})
        ET.SubElement(asset, "material", {"name": "floor_mat", "rgba": "0.25 0.25 0.25 1"})

        world = ET.SubElement(mj, "worldbody")
        ET.SubElement(world, "light", {"pos": "0 -3 4", "dir": "0 1 -1", "diffuse": "0.8 0.8 0.8"})
        ET.SubElement(world, "geom", {"name": "floor", "type": "plane", "size": "3 3 0.02", "material": "floor_mat"})
        ET.SubElement(world, "geom", {"name": "target_right", "type": "sphere", "pos": "-0.5 -0.15 1.0", "size": f"{self.target_radius}", "material": "target_right_mat", "contype": "0", "conaffinity": "0"})
        ET.SubElement(world, "geom", {"name": "target_left", "type": "sphere", "pos": "0.5 0.15 1.0", "size": f"{self.target_radius}", "material": "target_left_mat", "contype": "0", "conaffinity": "0"})
        for i in range(self.obstacle_count):
            ET.SubElement(world, "geom", {"name": f"obstacle_{i}", "type": "sphere", "pos": f"{0.25 + 0.15 * i} 0 1.0", "size": "0.1", "material": "obstacle_mat", "contype": "1", "conaffinity": "1"})

        body = ET.SubElement(world, "body", {"name": "base", "pos": "0 0 0"})
        ET.SubElement(
            body,
            "geom",
            {
                "name": "base_plate",
                "type": "box",
                "size": _fmt([dmm(40.0) / 2.0, dmm(40.0) / 2.0, dmm(10.0) / 2.0]),
                "pos": _fmt([0.0, 0.0, dmm(5.0)]),
                "material": "base_mat",
            },
        )
        ET.SubElement(
            body,
            "geom",
            {
                "name": "torso",
                "type": "box",
                "size": _fmt([dmm(28.0) / 2.0, dmm(28.0) / 2.0, L["L0"] / 2.0]),
                "pos": _fmt([0.0, _mm(self.config["base"].get("torso_y_offset", 0.0)), L["L0"] / 2.0]),
                "material": "torso_mat",
            },
        )
        ET.SubElement(body, "geom", {"name": "head", "type": "sphere", "size": f"{dmm(12.0):.8g}", "pos": _fmt([0.0, 0.0, L["L0"] + dmm(8.0)]), "material": "base_mat"})

        def add_arm(side: str) -> None:
            parent = body
            origins = self._joint_origins(side)
            axis_signs = self._joint_axis_signs(side)
            q_offset = self._arm_index(side) * 6
            for index, origin in enumerate(origins, start=1):
                name = f"{side}_joint_{index}"
                attrs = {"name": f"{name}_body"}
                attrs.update(_pose_attrs(origin))
                parent = ET.SubElement(parent, "body", attrs)
                ET.SubElement(
                    parent,
                    "joint",
                    {
                        "name": name,
                        "type": "hinge",
                        "axis": _fmt([0.0, 0.0, axis_signs[index - 1]]),
                        "limited": "true",
                        "range": f"{self.q_lower[q_offset + index - 1]:.8g} {self.q_upper[q_offset + index - 1]:.8g}",
                        "damping": "0.6",
                        "armature": "0.04",
                        "frictionloss": "0.0",
                    },
                )
                ET.SubElement(parent, "geom", {"type": "sphere", "size": f"{joint_radius:.8g}", "material": f"{side}_arm"})
                if index < len(origins):
                    end = origins[index][:3, 3]
                    length = float(np.linalg.norm(end))
                    if length > 1e-6:
                        ET.SubElement(parent, "geom", {"type": "capsule", "fromto": f"0 0 0 {_fmt(end)}", "size": f"{link_radius:.8g}", "material": "link"})

            tool_attrs = {"name": f"{side}_tool0_body"}
            tool_attrs.update(_pose_attrs(self._tool_matrix()))
            tool_body = ET.SubElement(parent, "body", tool_attrs)
            ET.SubElement(tool_body, "site", {"name": f"{side}_tool0", "pos": "0 0 0", "size": "0.035", "rgba": "1 0.2 0.05 1"})
            open_half_width = dmm(3.5)
            finger_z0 = -L["L7"] + dmm(5.5)
            for x in (open_half_width, -open_half_width):
                ET.SubElement(
                    tool_body,
                    "geom",
                    {
                        "type": "capsule",
                        "fromto": f"{x:.8g} 0 {finger_z0:.8g} {x:.8g} 0 0",
                        "size": f"{dmm(0.9):.8g}",
                        "material": "gripper_mat",
                    },
                )
            ET.SubElement(tool_body, "geom", {"type": "sphere", "size": f"{dmm(2.5):.8g}", "pos": "0 0 0", "rgba": "1 0.2 0.05 1"})

        for arm in self.arms:
            add_arm(arm)

        tendon = ET.SubElement(mj, "tendon")
        for arm in self.arms:
            fixed = ET.SubElement(tendon, "fixed", {"name": f"{arm}_q4_absolute"})
            ET.SubElement(fixed, "joint", {"joint": f"{arm}_joint_3", "coef": "1"})
            ET.SubElement(fixed, "joint", {"joint": f"{arm}_joint_4", "coef": "1"})

        equality = ET.SubElement(mj, "equality")
        ET.SubElement(
            equality,
            "joint",
            {
                "name": "shared_q1_equal",
                "joint1": "right_joint_1",
                "joint2": "left_joint_1",
                "polycoef": "0 1 0 0 0",
                "solref": "0.01 1",
                "solimp": "0.9 0.95 0.001",
            },
        )

        actuator = ET.SubElement(mj, "actuator")
        for arm in self.arms:
            q_offset = self._arm_index(arm) * 6
            for i in range(1, 7):
                attrs = {
                    "kp": "800",
                    "kv": "60",
                    "ctrllimited": "true",
                }
                if i == 3:
                    attrs.update(
                        {
                            "name": f"{arm}_q3_actuator_pos",
                            "joint": f"{arm}_joint_{i}",
                            "ctrlrange": f"{self.q_lower[q_offset + i - 1]:.8g} {self.q_upper[q_offset + i - 1]:.8g}",
                        }
                    )
                elif i == 4:
                    attrs.update(
                        {
                            "name": f"{arm}_q4_actuator_pos",
                            "tendon": f"{arm}_q4_absolute",
                            "ctrlrange": f"{self.ctrl_lower[q_offset + i - 1]:.8g} {self.ctrl_upper[q_offset + i - 1]:.8g}",
                        }
                    )
                else:
                    attrs.update(
                        {
                            "name": f"{arm}_joint_{i}_pos",
                            "joint": f"{arm}_joint_{i}",
                            "ctrlrange": f"{self.q_lower[q_offset + i - 1]:.8g} {self.q_upper[q_offset + i - 1]:.8g}",
                        }
                    )
                ET.SubElement(
                    actuator,
                    "position",
                    attrs,
                )

        xml_path.write_text(ET.tostring(mj, encoding="unicode"), encoding="utf-8")
        return xml_path

    def reset(self) -> np.ndarray:
        self.steps = 0
        self.prev_delta[:] = 0.0
        self.last_shielded = False
        self.shield_corrections = 0
        self._sample_domain()
        for _ in range(80):
            right = self.q_mid[self._arm_slice("right")] + self.rng.uniform(-0.12, 0.12, size=6) * self.q_range[self._arm_slice("right")]
            left = self._mirror_left_from_right(right)
            left += self.rng.uniform(-0.04, 0.04, size=6) * self.q_range[self._arm_slice("left")]
            self.q = self._enforce_q_coupling(np.concatenate([right, left]), q1_source="average")
            self.prev_q = self.q.copy()
            self.targets = self._sample_reachable_targets()
            self.obstacles = self._sample_obstacles(self.targets)
            if self._is_safe_q(self.q):
                break
        else:
            self.q = self._enforce_q_coupling(self.q_mid, q1_source="average")
            self.prev_q = self.q.copy()
            self.targets = self._sample_reachable_targets()
            self.obstacles = self._sample_obstacles(self.targets)
        self._sync_model()
        return self._obs()

    def _sample_reachable_targets(self) -> np.ndarray:
        span = 0.35 + 0.65 * self.curriculum_level
        for _ in range(200):
            right_slice = self._arm_slice("right")
            q_right = self.q_mid[right_slice] + self.rng.uniform(-0.5 * span, 0.5 * span, size=6) * self.q_range[right_slice]
            q_right = np.clip(q_right, self.q_lower[right_slice], self.q_upper[right_slice])
            q_left = self._mirror_left_from_right(q_right)
            q_pair = self._enforce_q_coupling(
                np.concatenate(
                    [
                        q_right,
                        np.clip(q_left, self.q_lower[self._arm_slice("left")], self.q_upper[self._arm_slice("left")]),
                    ]
                )
            )
            fk = self.fk(q_pair)
            targets = np.vstack([fk["right"][1], fk["left"][1]])
            if np.all((0.30 <= targets[:, 2]) & (targets[:, 2] <= 1.75)) and np.all(np.linalg.norm(targets[:, :2], axis=1) <= 1.55):
                self.target_q = q_pair.copy()
                return targets
        self.target_q = self._enforce_q_coupling(self.q_mid, q1_source="average")
        fk = self.fk(self.target_q)
        return np.vstack([fk["right"][1], fk["left"][1]])

    def _sample_obstacles(self, targets: np.ndarray) -> np.ndarray:
        zero_fk = self.fk(np.zeros(12, dtype=np.float64))
        obstacles = []
        level = self.curriculum_level
        for i in range(self.obstacle_count):
            arm = self.arms[i % len(self.arms)]
            target = targets[self._arm_index(arm)]
            shoulder = zero_fk[arm][0][2]
            direction = target - shoulder
            norm = np.linalg.norm(direction)
            if norm < 1e-6:
                direction = np.array([-1.0 if arm == "right" else 1.0, 0.0, 0.0])
            else:
                direction = direction / norm
            side = np.cross(direction, np.array([0.0, 0.0, 1.0]))
            if np.linalg.norm(side) < 1e-6:
                side = np.array([0.0, -1.0 if arm == "right" else 1.0, 0.0])
            side = side / np.linalg.norm(side)
            frac = self.rng.uniform(0.32, 0.78)
            lateral_min = max(0.035, 0.18 - 0.12 * level)
            lateral_max = max(lateral_min + 0.03, 0.30 - 0.12 * level)
            lateral = self.rng.choice([-1.0, 1.0]) * self.rng.uniform(lateral_min, lateral_max)
            vertical = self.rng.uniform(-0.08, 0.08)
            radius = self.rng.uniform(0.05 + 0.02 * level, 0.08 + 0.05 * level)
            center = shoulder + frac * (target - shoulder) + lateral * side + np.array([0.0, 0.0, vertical])
            obstacles.append([center[0], center[1], max(0.12, center[2]), radius])
        return np.array(obstacles, dtype=np.float64)

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict]:
        raw_action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        action = self._apply_action_domain(raw_action)
        self.prev_q = self.q.copy()
        delta = self.action_scale * action
        max_delta = np.minimum(self.action_scale, self.q_velocity_limits * self.dt)
        delta = np.clip(delta, -max_delta, max_delta)
        max_delta_change = np.deg2rad(5.0 + 8.0 * self.curriculum_level)
        delta = self.prev_delta + np.clip(delta - self.prev_delta, -max_delta_change, max_delta_change)
        shared_q1_delta = 0.5 * (delta[self._arm_slice("right").start] + delta[self._arm_slice("left").start])
        delta[self._arm_slice("right").start] = shared_q1_delta
        delta[self._arm_slice("left").start] = shared_q1_delta
        q_goal = self._enforce_q_coupling(self.q + delta, q1_source="average")
        self.q, self.last_shielded = self._safety_project(self.q, q_goal)
        self.prev_delta = self.q - self.prev_q
        if self.last_shielded:
            self.shield_corrections += 1
            self.total_shield_corrections += 1
        self.steps += 1
        self._sync_model()
        info = self._reach_info()
        info.shielded = self.last_shielded
        reward = self._reward(raw_action, info)
        done = bool(info.success or info.collision or self.steps >= self.max_steps)
        return self._obs(info), float(reward), done, info.__dict__.copy()

    def _sync_model(self) -> None:
        self.data.qpos[self.qpos_addr] = self.q
        self.data.qvel[self.qvel_addr] = (self.q - self.prev_q) / self.dt
        self.data.ctrl[:12] = self._q_to_ctrl(self.q)
        for geom_id, target in zip(self.target_geoms, self.targets):
            self.model.geom_pos[geom_id] = target
            self.model.geom_size[geom_id, 0] = self.target_radius
        for geom_id, obstacle in zip(self.obstacle_geoms, self.obstacles):
            self.model.geom_pos[geom_id] = obstacle[:3]
            self.model.geom_size[geom_id, 0] = obstacle[3]
        mujoco.mj_forward(self.model, self.data)

    def _reach_info(self, q: np.ndarray | None = None) -> ReachInfo:
        q_eval = self.q if q is None else np.asarray(q, dtype=np.float64)
        fk = self.fk(q_eval)
        tools = np.vstack([fk[arm][1] for arm in self.arms])
        rotations = [fk[arm][2] for arm in self.arms]
        distances = np.linalg.norm(tools - self.targets, axis=1)
        clearances = []
        for obstacle in self.obstacles:
            center, radius = obstacle[:3], float(obstacle[3])
            for arm in self.arms:
                points = fk[arm][0]
                for a, b in zip(points[:-1], points[1:]):
                    clearances.append(_segment_sphere_clearance(a, b, center, radius))
        right_points = fk["right"][0]
        left_points = fk["left"][0]
        link_radius = _mm(self.config["visual"].get("link_radius", 60.0))
        for a0, a1 in zip(right_points[3:-1], right_points[4:]):
            for b0, b1 in zip(left_points[3:-1], left_points[4:]):
                clearances.append(_segment_segment_distance(a0, a1, b0, b1) - 2.0 * link_radius)
        for arm in self.arms:
            clearances.extend(self._torso_link_clearances(fk[arm][0]))
        min_clearance = min(clearances) if clearances else 1.0
        distance = float(np.mean(distances))
        parallel = float(np.mean([abs(np.dot(R[:, 2], np.array([0.0, 0.0, 1.0]))) for R in rotations]))
        q_right = q_eval[self._arm_slice("right")]
        q_left = q_eval[self._arm_slice("left")]
        coordination_error = np.linalg.norm((q_left - self._mirror_left_from_right(q_right)) / np.maximum(self.q_range[self._arm_slice("left")], 1e-6))
        coordination = float(math.exp(-2.0 * coordination_error))
        collision = min_clearance < 0.0
        success = bool(np.all(distances < self.target_radius) and min_clearance > self.clearance_margin and parallel > 0.85)
        return ReachInfo(distance, float(min_clearance), parallel, coordination, success, collision)

    def _reward(self, action: np.ndarray, info: ReachInfo) -> float:
        reach = 6.0 * math.exp(-8.0 * info.distance)
        distance_penalty = -2.0 * info.distance
        parallel = 1.2 * info.parallel_score + 2.0 * info.parallel_score * math.exp(-10.0 * info.distance)
        coordination = 0.8 * info.coordination_score
        clearance = 0.6 * math.tanh(max(info.min_clearance, -0.2) / self.clearance_margin)
        action_penalty = -0.015 * float(np.dot(action, action))
        mirrored_action = self._mirror_left_from_right(action[:6])
        action_sync = 0.25 * math.exp(-0.5 * float(np.linalg.norm(action[6:] - mirrored_action)))
        limit_margin = np.minimum(self.q - self.q_lower, self.q_upper - self.q)
        limit_penalty = -0.1 * float(np.sum(limit_margin < np.deg2rad(2.0)))
        shield_penalty = -0.35 if info.shielded else 0.0
        reward = reach + distance_penalty + parallel + coordination + action_sync + clearance + action_penalty + limit_penalty + shield_penalty
        if info.collision:
            reward -= 12.0
        if info.success:
            reward += 25.0
        return reward

    def _obs(self, info: ReachInfo | None = None) -> np.ndarray:
        q_obs = self.q.copy()
        targets_obs = self.targets.copy()
        obstacles_obs = self.obstacles.copy()
        if self.domain_randomization:
            q_obs += self.rng.normal(0.0, self.obs_q_noise_std, size=12)
            targets_obs += self.rng.normal(0.0, self.obs_target_noise_std, size=targets_obs.shape)
            obstacles_obs[:, :3] += self.rng.normal(0.0, self.obs_obstacle_noise_std, size=obstacles_obs[:, :3].shape)
            obstacles_obs[:, 3] = np.clip(
                obstacles_obs[:, 3] + self.rng.normal(0.0, 0.004 * self.curriculum_level, size=len(obstacles_obs)),
                0.04,
                0.16,
            )
        q_obs = self._enforce_q_coupling(q_obs, q1_source="average")
        fk = self.fk(q_obs)
        tools = np.vstack([fk[arm][1] for arm in self.arms])
        tool_axes = np.concatenate([fk[arm][2][:, 2] for arm in self.arms])
        q_norm = 2.0 * (q_obs - self.q_lower) / self.q_range - 1.0
        qvel = np.clip((q_obs - self.prev_q) / self.action_scale, -5.0, 5.0) / 5.0
        tool_to_target = (targets_obs - tools).reshape(-1)
        rel_obs = []
        radii = []
        for obstacle in obstacles_obs:
            for tool in tools:
                rel_obs.extend((obstacle[:3] - tool) / 1.5)
            radii.append(obstacle[3] / 0.2)
        if info is None:
            info = self._reach_info()
        obs = np.concatenate(
            [
                q_norm,
                qvel,
                tool_to_target / 1.5,
                tools.reshape(-1) / 1.5,
                tool_axes,
                np.array(rel_obs, dtype=np.float64),
                np.array(radii, dtype=np.float64),
                np.array([info.min_clearance / 0.3, info.parallel_score, info.coordination_score], dtype=np.float64),
            ]
        )
        return obs.astype(np.float32)

    def render(self) -> None:
        import mujoco.viewer

        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.distance = 2.8
            self.viewer.cam.azimuth = 135
            self.viewer.cam.elevation = -25
            self.viewer.cam.lookat[:] = np.array([0.25, 0.0, 1.0])
        self.viewer.sync()

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
