"""Generate planner-teacher BC data for AGV reach PPO."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from rl_mujoco.agv_reach_env import AgvReachEnv, DEFAULT_GEOMETRY


def generate(args) -> dict:
    rng = np.random.default_rng(args.seed)
    env = AgvReachEnv(
        args.geometry,
        seed=args.seed,
        max_steps=args.max_traj_len,
        obstacle_count=args.obstacles,
        render=False,
        domain_randomization=args.domain_randomization,
        safety_shield=True,
    )

    obs_samples: list[np.ndarray] = []
    action_samples: list[np.ndarray] = []
    successes = 0
    collisions = 0
    planned = 0
    kept_episodes = 0
    path_lengths = []
    final_distances = []
    final_parallels = []
    final_clearances = []
    curriculum_levels = []
    curriculum_bins = np.zeros(5, dtype=np.int64)
    curriculum_success_bins = np.zeros(5, dtype=np.int64)

    for episode in range(1, args.episodes + 1):
        if args.curriculum < 0.0:
            u = float(rng.uniform(0.0, 1.0))
            power = max(float(args.curriculum_power), 1e-6)
            level = float(args.min_curriculum + (args.max_curriculum - args.min_curriculum) * (u ** power))
        else:
            level = float(args.curriculum)
        level = float(np.clip(level, 0.0, 1.0))
        curriculum_levels.append(level)
        bin_index = min(4, max(0, int(level * 5.0)))
        curriculum_bins[bin_index] += 1
        env.set_curriculum(level)
        obs = env.reset()
        path = env.plan_to_target(max_nodes=args.planner_nodes)
        env._teacher_path = path
        env._teacher_path_index = 1
        env._teacher_target_q = env.target_q.copy()
        path_lengths.append(len(path))
        planned += int(len(path) > 1)
        done = False
        info = None
        episode_obs: list[np.ndarray] = []
        episode_actions: list[np.ndarray] = []

        for _ in range(args.max_traj_len):
            action = env.teacher_action()
            episode_obs.append(obs.copy())
            episode_actions.append(action.astype(np.float32, copy=True))
            obs, _, done, info = env.step(action)
            if done:
                break

        if info is None:
            info = env._reach_info().__dict__.copy()
        success = bool(info["success"])
        collision = bool(info["collision"])
        successes += int(success)
        curriculum_success_bins[bin_index] += int(success)
        collisions += int(collision)
        final_distances.append(float(info["distance"]))
        final_parallels.append(float(info["parallel_score"]))
        final_clearances.append(float(info["min_clearance"]))
        if success or args.keep_failures:
            obs_samples.extend(episode_obs)
            action_samples.extend(episode_actions)
            kept_episodes += 1

        if episode == 1 or episode % max(1, args.log_freq) == 0:
            print(
                f"episode={episode:05d} kept={kept_episodes:05d} samples={len(obs_samples)} "
                f"success={successes / episode:5.2%} collision={collisions / episode:5.2%} "
                f"mean_distance={float(np.mean(final_distances)):.3f} "
                f"mean_parallel={float(np.mean(final_parallels)):.3f} "
                f"mean_path_len={float(np.mean(path_lengths)):.1f}"
            )

    env.close()
    obs_arr = np.asarray(obs_samples, dtype=np.float32)
    action_arr = np.asarray(action_samples, dtype=np.float32)
    if len(obs_arr) == 0:
        raise RuntimeError("Teacher dataset is empty. Lower curriculum range or inspect planner failures.")
    metadata = {
        "episodes": args.episodes,
        "kept_episodes": kept_episodes,
        "samples": int(len(obs_arr)),
        "success_rate": successes / max(1, args.episodes),
        "collision_rate": collisions / max(1, args.episodes),
        "planned_rate": planned / max(1, args.episodes),
        "mean_path_len": float(np.mean(path_lengths)) if path_lengths else 0.0,
        "mean_final_distance": float(np.mean(final_distances)) if final_distances else 0.0,
        "mean_final_parallel": float(np.mean(final_parallels)) if final_parallels else 0.0,
        "mean_final_clearance": float(np.mean(final_clearances)) if final_clearances else 0.0,
        "keep_failures": args.keep_failures,
        "max_traj_len": args.max_traj_len,
        "obstacles": args.obstacles,
        "domain_randomization": args.domain_randomization,
        "curriculum": args.curriculum,
        "min_curriculum": args.min_curriculum,
        "max_curriculum": args.max_curriculum,
        "curriculum_power": args.curriculum_power,
        "mean_curriculum": float(np.mean(curriculum_levels)) if curriculum_levels else 0.0,
        "curriculum_bins": curriculum_bins.tolist(),
        "curriculum_success_bins": curriculum_success_bins.tolist(),
        "curriculum_success_rate_bins": (
            curriculum_success_bins / np.maximum(curriculum_bins, 1)
        ).astype(float).tolist(),
    }
    return {"obs": obs_arr, "actions": action_arr, "levels": np.asarray(curriculum_levels, dtype=np.float32), "metadata": metadata}


def parse_args():
    parser = argparse.ArgumentParser(description="Generate AGV planner-teacher BC dataset")
    parser.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument("--out", type=Path, default=Path("/tmp/agv_reach_teacher/agv_reach_teacher.npz"))
    parser.add_argument("--episodes", type=int, default=800)
    parser.add_argument("--max-traj-len", type=int, default=160)
    parser.add_argument("--obstacles", type=int, default=2)
    parser.add_argument("--seed", type=int, default=7007)
    parser.add_argument("--planner-nodes", type=int, default=700)
    parser.add_argument("--curriculum", type=float, default=-1.0, help="Use <0 to sample a curriculum range.")
    parser.add_argument("--min-curriculum", type=float, default=0.0)
    parser.add_argument("--max-curriculum", type=float, default=1.0)
    parser.add_argument("--curriculum-power", type=float, default=1.0)
    parser.add_argument("--domain-randomization", action="store_true")
    parser.add_argument("--keep-failures", action="store_true")
    parser.add_argument("--log-freq", type=int, default=50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = generate(args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        obs=result["obs"],
        actions=result["actions"],
        curriculum_levels=result["levels"],
        metadata=json.dumps(result["metadata"], indent=2),
    )
    print(f"saved={args.out}")
    print(json.dumps(result["metadata"], indent=2))


if __name__ == "__main__":
    main()
