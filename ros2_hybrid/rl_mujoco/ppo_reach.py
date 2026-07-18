"""PPO trainer/viewer for AGV arm reach-and-avoid MuJoCo task."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

from rl_mujoco.agv_reach_env import AgvReachEnv, DEFAULT_GEOMETRY

try:
    import ray
except ImportError:  # pragma: no cover - exercised only when Ray is not installed.
    ray = None


class Actor(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int, init_std: float = 0.45):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, action_dim),
        )
        self.log_std = nn.Parameter(torch.full((action_dim,), float(np.log(init_std))))
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.ones(obs_dim))

    def _norm(self, obs: torch.Tensor) -> torch.Tensor:
        return (obs - self.obs_mean) / torch.clamp(self.obs_std, min=1e-6)

    def distribution(self, obs: torch.Tensor) -> Normal:
        mean = torch.tanh(self.net(self._norm(obs)))
        std = torch.exp(self.log_std).expand_as(mean)
        return Normal(mean, std)

    def act(self, obs: torch.Tensor, deterministic: bool = False):
        dist = self.distribution(obs)
        raw = dist.mean if deterministic else dist.rsample()
        action = torch.clamp(raw, -1.0, 1.0)
        log_prob = dist.log_prob(raw).sum(-1)
        return action, log_prob


class Critic(nn.Module):
    def __init__(self, obs_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 128),
            nn.Tanh(),
            nn.Linear(128, 1),
        )
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.ones(obs_dim))

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        obs = (obs - self.obs_mean) / torch.clamp(self.obs_std, min=1e-6)
        return self.net(obs).squeeze(-1)


def latest_run(logdir: Path) -> Path:
    runs = [p for p in logdir.iterdir() if p.is_dir() and (p / "actor.pt").exists()] if logdir.exists() else []
    if not runs:
        raise FileNotFoundError(f"No actor.pt checkpoints found under {logdir}")
    return max(runs, key=lambda p: p.stat().st_mtime)


def save_checkpoint(path: Path, actor: Actor, critic: Critic, args, obs_dim: int, action_dim: int) -> None:
    torch.save(
        {
            "actor_state": actor.state_dict(),
            "critic_state": critic.state_dict(),
            "obs_dim": obs_dim,
            "action_dim": action_dim,
            "std_dev": args.std_dev,
        },
        path,
    )


def load_actor(path: Path) -> Actor:
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if isinstance(checkpoint, Actor):
        checkpoint.eval()
        return checkpoint
    actor = Actor(
        int(checkpoint["obs_dim"]),
        int(checkpoint["action_dim"]),
        float(checkpoint.get("std_dev", 0.45)),
    )
    actor.load_state_dict(checkpoint["actor_state"])
    actor.eval()
    return actor


def compute_gae(rewards, dones, values, next_values, gamma: float, lam: float):
    advantages = np.zeros_like(rewards, dtype=np.float32)
    last_gae = np.zeros(rewards.shape[1], dtype=np.float32)
    for t in reversed(range(rewards.shape[0])):
        non_terminal = 1.0 - dones[t].astype(np.float32)
        delta = rewards[t] + gamma * next_values[t] * non_terminal - values[t]
        last_gae = delta + gamma * lam * non_terminal * last_gae
        advantages[t] = last_gae
    returns = advantages + values
    return advantages.reshape(-1), returns.reshape(-1)


def make_env(args, seed: int, *, train_mode: bool, render: bool = False) -> AgvReachEnv:
    domain_randomization = train_mode and not getattr(args, "no_domain_randomization", False)
    if getattr(args, "eval_domain_randomization", False):
        domain_randomization = True
    return AgvReachEnv(
        args.geometry,
        seed=seed,
        max_steps=args.max_traj_len,
        obstacle_count=args.obstacles,
        render=render,
        domain_randomization=domain_randomization,
        safety_shield=not getattr(args, "no_safety_shield", False),
    )


def _worker_args(args) -> dict:
    return {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}


def _cpu_state_dict(module: nn.Module) -> dict[str, torch.Tensor]:
    return {key: value.detach().cpu() for key, value in module.state_dict().items()}


if ray is not None:

    @ray.remote
    class AgvRolloutWorker:
        """Persistent Ray worker that owns one MuJoCo environment."""

        def __init__(self, args_dict: dict, seed: int, worker_id: int):
            self.args = argparse.Namespace(**args_dict)
            self.args.geometry = Path(self.args.geometry)
            self.seed = int(seed)
            self.worker_id = int(worker_id)
            torch.set_num_threads(1)
            torch.manual_seed(self.seed)
            np.random.seed(self.seed)
            self.env = make_env(self.args, self.seed, train_mode=True)
            self.actor = Actor(self.env.obs_dim, self.env.action_dim, self.args.std_dev)
            self.critic = Critic(self.env.obs_dim)
            self.obs: np.ndarray | None = None

        def sync(self, actor_state: dict, critic_state: dict, curriculum: float) -> None:
            self.actor.load_state_dict(actor_state)
            self.critic.load_state_dict(critic_state)
            self.actor.eval()
            self.critic.eval()
            self.env.set_curriculum(float(curriculum))

        @torch.no_grad()
        def sample(self, steps: int) -> dict:
            steps = int(steps)
            obs_dim = self.env.obs_dim
            action_dim = self.env.action_dim
            obs_buf = np.empty((steps, obs_dim), dtype=np.float32)
            act_buf = np.empty((steps, action_dim), dtype=np.float32)
            logp_buf = np.empty((steps,), dtype=np.float32)
            rew_buf = np.empty((steps,), dtype=np.float32)
            done_buf = np.empty((steps,), dtype=np.bool_)
            val_buf = np.empty((steps,), dtype=np.float32)
            next_val_buf = np.empty((steps,), dtype=np.float32)
            ep_success: list[float] = []
            ep_collision: list[float] = []

            if self.obs is None:
                self.obs = self.env.reset()

            for t in range(steps):
                obs_now = np.asarray(self.obs, dtype=np.float32)
                obs_t = torch.tensor(obs_now[None, :], dtype=torch.float32)
                action_t, logp_t = self.actor.act(obs_t)
                value_t = self.critic(obs_t)
                action = action_t.numpy()[0]
                next_obs, reward, done, info = self.env.step(action)
                next_obs_arr = np.asarray(next_obs, dtype=np.float32)
                next_value = self.critic(torch.tensor(next_obs_arr[None, :], dtype=torch.float32)).numpy()[0]

                obs_buf[t] = obs_now
                act_buf[t] = action
                logp_buf[t] = logp_t.numpy()[0]
                rew_buf[t] = float(reward)
                done_buf[t] = bool(done)
                val_buf[t] = value_t.numpy()[0]
                next_val_buf[t] = float(next_value)

                if done:
                    ep_success.append(float(info["success"]))
                    ep_collision.append(float(info["collision"]))
                    self.obs = self.env.reset()
                else:
                    self.obs = next_obs_arr

            return {
                "obs": obs_buf,
                "actions": act_buf,
                "logp": logp_buf,
                "rewards": rew_buf,
                "dones": done_buf,
                "values": val_buf,
                "next_values": next_val_buf,
                "ep_success": ep_success,
                "ep_collision": ep_collision,
            }

        def close(self) -> None:
            self.env.close()


def behavior_clone(actor: Actor, envs: list[AgvReachEnv], args) -> None:
    if args.bc_steps <= 0:
        return
    optimizer = torch.optim.Adam(actor.parameters(), lr=args.bc_lr, eps=1e-5)
    obs = np.stack([env.reset() for env in envs])
    print(f"BC warm-start: steps={args.bc_steps} batch={len(envs)}")
    for step in range(1, args.bc_steps + 1):
        level = min(1.0, step / max(1, args.bc_steps))
        for env in envs:
            env.set_curriculum(level)
        teacher_actions = np.stack([env.teacher_action() for env in envs]).astype(np.float32)
        obs_tensor = torch.tensor(obs, dtype=torch.float32)
        action_tensor = torch.tensor(teacher_actions, dtype=torch.float32)
        pred = actor.distribution(obs_tensor).mean
        loss = (pred - action_tensor).pow(2).mean()
        optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(actor.parameters(), args.max_grad_norm)
        optimizer.step()

        next_obs = []
        for env, teacher_action in zip(envs, teacher_actions):
            ob, _, done, _ = env.step(teacher_action)
            next_obs.append(env.reset() if done else ob)
        obs = np.stack(next_obs)
        if step == 1 or step % max(1, args.bc_log_freq) == 0:
            print(f"bc_step={step:05d} loss={float(loss):.5f}")


@torch.no_grad()
def evaluate_actor(actor: Actor, args, *, episodes: int, domain_randomization: bool) -> dict:
    eval_args = argparse.Namespace(**vars(args))
    eval_args.eval_domain_randomization = domain_randomization
    eval_args.no_domain_randomization = not domain_randomization
    env = make_env(eval_args, args.seed + 100_000, train_mode=False, render=False)
    successes, collisions, distances, clearances, parallels, shield_rates = [], [], [], [], [], []
    for episode in range(episodes):
        env.set_curriculum(1.0)
        obs = env.reset()
        shielded_steps = 0
        final_info = None
        for _ in range(args.max_traj_len):
            action, _ = actor.act(torch.tensor(obs[None, :], dtype=torch.float32), deterministic=True)
            obs, _, done, info = env.step(action.numpy()[0])
            shielded_steps += int(info.get("shielded", False))
            final_info = info
            if done:
                break
        if final_info is None:
            final_info = env._reach_info().__dict__.copy()
        successes.append(float(final_info["success"]))
        collisions.append(float(final_info["collision"]))
        distances.append(float(final_info["distance"]))
        clearances.append(float(final_info["min_clearance"]))
        parallels.append(float(final_info["parallel_score"]))
        shield_rates.append(shielded_steps / max(1, env.steps))
    env.close()
    return {
        "episodes": episodes,
        "domain_randomization": domain_randomization,
        "success_rate": float(np.mean(successes)),
        "collision_rate": float(np.mean(collisions)),
        "mean_distance": float(np.mean(distances)),
        "mean_clearance": float(np.mean(clearances)),
        "mean_parallel": float(np.mean(parallels)),
        "mean_shield_rate": float(np.mean(shield_rates)),
    }


def collect_rollouts_sequential(
    actor: Actor,
    critic: Critic,
    envs: list[AgvReachEnv],
    obs: np.ndarray,
    args,
    level: float,
    obs_dim: int,
    action_dim: int,
) -> tuple[dict, np.ndarray]:
    for env in envs:
        env.set_curriculum(level)
    obs_buf, act_buf, logp_buf, rew_buf, done_buf, val_buf, next_val_buf = [], [], [], [], [], [], []
    ep_success = []
    ep_collision = []
    for _ in range(args.max_traj_len):
        obs_t = torch.tensor(obs, dtype=torch.float32)
        with torch.no_grad():
            actions_t, logp_t = actor.act(obs_t)
            values_t = critic(obs_t)
        actions = actions_t.numpy()
        next_obs, rewards, dones = [], [], []
        for i, env in enumerate(envs):
            ob, rew, done, info = env.step(actions[i])
            rewards.append(rew)
            dones.append(done)
            if done:
                ep_success.append(float(info["success"]))
                ep_collision.append(float(info["collision"]))
                ob = env.reset()
            next_obs.append(ob)
        next_obs = np.stack(next_obs)
        with torch.no_grad():
            next_values = critic(torch.tensor(next_obs, dtype=torch.float32)).numpy()
        obs_buf.append(obs.copy())
        act_buf.append(actions.copy())
        logp_buf.append(logp_t.numpy().copy())
        rew_buf.append(np.array(rewards, dtype=np.float32))
        done_buf.append(np.array(dones, dtype=np.bool_))
        val_buf.append(values_t.numpy().copy())
        next_val_buf.append(next_values.copy())
        obs = next_obs

    data = {
        "obs_arr": np.asarray(obs_buf, dtype=np.float32).reshape(-1, obs_dim),
        "act_arr": np.asarray(act_buf, dtype=np.float32).reshape(-1, action_dim),
        "old_logp": np.asarray(logp_buf, dtype=np.float32).reshape(-1),
        "rewards": np.asarray(rew_buf, dtype=np.float32),
        "dones": np.asarray(done_buf, dtype=np.bool_),
        "values": np.asarray(val_buf, dtype=np.float32),
        "next_values": np.asarray(next_val_buf, dtype=np.float32),
        "ep_success": ep_success,
        "ep_collision": ep_collision,
    }
    return data, obs


def collect_rollouts_ray(
    workers,
    actor: Actor,
    critic: Critic,
    args,
    level: float,
    obs_dim: int,
    action_dim: int,
) -> dict:
    actor_ref = ray.put(_cpu_state_dict(actor))
    critic_ref = ray.put(_cpu_state_dict(critic))
    ray.get([worker.sync.remote(actor_ref, critic_ref, level) for worker in workers])
    results = ray.get([worker.sample.remote(args.max_traj_len) for worker in workers])

    obs_seq = np.stack([result["obs"] for result in results], axis=1)
    act_seq = np.stack([result["actions"] for result in results], axis=1)
    logp_seq = np.stack([result["logp"] for result in results], axis=1)
    rewards = np.stack([result["rewards"] for result in results], axis=1)
    dones = np.stack([result["dones"] for result in results], axis=1)
    values = np.stack([result["values"] for result in results], axis=1)
    next_values = np.stack([result["next_values"] for result in results], axis=1)

    return {
        "obs_arr": obs_seq.astype(np.float32, copy=False).reshape(-1, obs_dim),
        "act_arr": act_seq.astype(np.float32, copy=False).reshape(-1, action_dim),
        "old_logp": logp_seq.astype(np.float32, copy=False).reshape(-1),
        "rewards": rewards.astype(np.float32, copy=False),
        "dones": dones.astype(np.bool_, copy=False),
        "values": values.astype(np.float32, copy=False),
        "next_values": next_values.astype(np.float32, copy=False),
        "ep_success": [value for result in results for value in result["ep_success"]],
        "ep_collision": [value for result in results for value in result["ep_collision"]],
    }


def train(args):
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    run_dir = Path(args.logdir) / f"{datetime.now().strftime('%y-%m-%d-%H-%M-%S')}_agv_reach"
    run_dir.mkdir(parents=True, exist_ok=True)
    use_ray = bool(not args.no_ray_workers and args.num_procs > 1 and ray is not None)
    if not args.no_ray_workers and args.num_procs > 1 and ray is None:
        print("Ray is not installed; falling back to sequential rollout.")
    local_env_count = args.num_procs if (args.bc_steps > 0 or not use_ray) else 1
    envs = [
        make_env(args, args.seed + i, train_mode=True)
        for i in range(local_env_count)
    ]
    obs_dim, action_dim = envs[0].obs_dim, envs[0].action_dim
    actor = Actor(obs_dim, action_dim, args.std_dev)
    critic = Critic(obs_dim)
    optimizer = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=args.lr, eps=1e-5)

    config = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    config["geometry"] = str(args.geometry)
    (run_dir / "config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    (run_dir / "model.xml").write_text(Path(envs[0].xml_path).read_text(encoding="utf-8"), encoding="utf-8")

    behavior_clone(actor, envs, args)
    save_checkpoint(run_dir / "actor_bc.pt", actor, critic, args, obs_dim, action_dim)
    save_checkpoint(run_dir / "actor.pt", actor, critic, args, obs_dim, action_dim)
    print(f"Saved BC warm-start actor: {run_dir / 'actor_bc.pt'}")
    workers = []
    obs = None
    if use_ray:
        if not ray.is_initialized():
            ray.init(num_cpus=args.num_procs, include_dashboard=False, log_to_driver=False)
        worker_args = _worker_args(args)
        workers = [
            AgvRolloutWorker.remote(worker_args, args.seed + 10_000 + i, i)
            for i in range(args.num_procs)
        ]
        print(f"Using Ray rollout workers: num_workers={len(workers)}")
        for env in envs:
            env.close()
        envs = []
    else:
        obs = np.stack([env.reset() for env in envs])
        print(f"Using sequential rollout envs: num_envs={len(envs)}")
    best_success = 0.0
    print(f"Training AGV reach policy: logdir={run_dir}")
    for itr in range(1, args.n_itr + 1):
        level = min(1.0, itr / max(1, args.curriculum_iters)) if args.curriculum_iters > 0 else 1.0
        if use_ray:
            rollout = collect_rollouts_ray(workers, actor, critic, args, level, obs_dim, action_dim)
        else:
            rollout, obs = collect_rollouts_sequential(actor, critic, envs, obs, args, level, obs_dim, action_dim)

        obs_arr = rollout["obs_arr"]
        act_arr = rollout["act_arr"]
        old_logp = rollout["old_logp"]
        rewards = rollout["rewards"]
        dones = rollout["dones"]
        values = rollout["values"]
        next_values = rollout["next_values"]
        adv, ret = compute_gae(rewards, dones, values, next_values, args.gamma, args.lam)
        adv = (adv - adv.mean()) / (adv.std() + 1e-6)

        obs_tensor = torch.tensor(obs_arr, dtype=torch.float32)
        act_tensor = torch.tensor(act_arr, dtype=torch.float32)
        old_logp_tensor = torch.tensor(old_logp, dtype=torch.float32)
        adv_tensor = torch.tensor(adv, dtype=torch.float32)
        ret_tensor = torch.tensor(ret, dtype=torch.float32)

        inds = np.arange(len(obs_arr))
        for _ in range(args.epochs):
            np.random.shuffle(inds)
            for start in range(0, len(inds), args.minibatch_size):
                mb = inds[start : start + args.minibatch_size]
                dist = actor.distribution(obs_tensor[mb])
                logp = dist.log_prob(act_tensor[mb]).sum(-1)
                ratio = torch.exp(logp - old_logp_tensor[mb])
                clipped = torch.clamp(ratio, 1.0 - args.clip, 1.0 + args.clip) * adv_tensor[mb]
                policy_loss = -torch.min(ratio * adv_tensor[mb], clipped).mean()
                value_loss = 0.5 * (critic(obs_tensor[mb]) - ret_tensor[mb]).pow(2).mean()
                entropy = dist.entropy().sum(-1).mean()
                loss = policy_loss + args.value_coeff * value_loss - args.entropy_coeff * entropy
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()), args.max_grad_norm)
                optimizer.step()

        success_rate = float(np.mean(rollout["ep_success"])) if rollout["ep_success"] else 0.0
        collision_rate = float(np.mean(rollout["ep_collision"])) if rollout["ep_collision"] else 0.0
        mean_reward = float(rewards.sum(axis=0).mean())
        if itr % args.eval_freq == 0 or itr == 1:
            print(
                f"itr={itr:05d} reward={mean_reward:8.3f} "
                f"success={success_rate:5.2%} collision={collision_rate:5.2%} curriculum={level:4.2f}"
            )
            if args.eval_episodes > 0:
                eval_metrics = evaluate_actor(actor, args, episodes=args.eval_episodes, domain_randomization=True)
                print(
                    "  robust_eval "
                    f"success={eval_metrics['success_rate']:5.2%} "
                    f"collision={eval_metrics['collision_rate']:5.2%} "
                    f"clearance={eval_metrics['mean_clearance']:.3f} "
                    f"shield={eval_metrics['mean_shield_rate']:5.2%}"
                )
        if success_rate >= best_success or itr % args.save_freq == 0:
            best_success = max(best_success, success_rate)
            save_checkpoint(run_dir / "actor.pt", actor, critic, args, obs_dim, action_dim)
        if itr % args.save_freq == 0:
            save_checkpoint(run_dir / f"actor_{itr}.pt", actor, critic, args, obs_dim, action_dim)

    save_checkpoint(run_dir / "actor.pt", actor, critic, args, obs_dim, action_dim)
    if workers:
        ray.get([worker.close.remote() for worker in workers])
    for env in envs:
        env.close()
    if args.gate_episodes > 0:
        gate = evaluate_actor(actor, args, episodes=args.gate_episodes, domain_randomization=True)
        gate["required_success_rate"] = args.gate_success
        gate["max_collision_rate"] = args.gate_collision
        gate["real_ready"] = bool(
            gate["success_rate"] >= args.gate_success and gate["collision_rate"] <= args.gate_collision
        )
        (run_dir / "real_gate.json").write_text(json.dumps(gate, indent=2), encoding="utf-8")
        print(
            "Real gate: "
            f"success={gate['success_rate']:5.2%}/{args.gate_success:5.2%} "
            f"collision={gate['collision_rate']:5.2%}/{args.gate_collision:5.2%} "
            f"ready={gate['real_ready']}"
        )
    else:
        print("Real gate skipped: gate_episodes=0")
    print(f"Training complete. Saved actor: {run_dir / 'actor.pt'}")


@torch.no_grad()
def eval_policy(args):
    run_dir = Path(args.path) if args.path else latest_run(Path(args.logdir))
    actor_path = run_dir if run_dir.is_file() else run_dir / "actor.pt"
    if not actor_path.exists():
        raise FileNotFoundError(f"Missing actor checkpoint: {actor_path}")
    actor = load_actor(actor_path)
    env = make_env(args, args.seed, train_mode=False, render=True)
    obs = env.reset()
    print(f"Viewing policy: {actor_path}")
    print("Close the MuJoCo viewer window to stop.")
    while True:
        start = time.time()
        info = None
        done = False
        for _ in range(max(1, int(args.view_speed))):
            obs_t = torch.tensor(obs[None, :], dtype=torch.float32)
            action, _ = actor.act(obs_t, deterministic=True)
            obs, _, done, info = env.step(action.numpy()[0])
            if done:
                break
        env.render()
        if done:
            print(
                f"episode done: success={info['success']} collision={info['collision']} "
                f"distance={info['distance']:.3f} clearance={info['min_clearance']:.3f} "
                f"parallel={info['parallel_score']:.3f}"
            )
            obs = env.reset()
        if env.viewer is not None and not env.viewer.is_running():
            break
        if not args.no_realtime:
            time.sleep(max(0.0, env.dt - (time.time() - start)))
    env.close()


def inspect_joints(args):
    import mujoco
    import mujoco.viewer

    env = make_env(args, args.seed, train_mode=False, render=True)
    env.q = np.zeros(12, dtype=np.float64)
    env.prev_q = env.q.copy()
    fk = env.fk(env.q)
    tools = np.vstack([fk[arm][1] for arm in env.arms])
    env.targets = tools.copy()
    for i in range(env.obstacle_count):
        side_y = -0.45 if i % 2 == 0 else 0.45
        env.obstacles[i] = np.array([0.45 + 0.12 * i, side_y, 0.85, 0.08], dtype=np.float64)
    env._sync_model()

    print(f"Inspecting AGV MuJoCo joints: xml={env.xml_path}")
    for arm, tool in zip(env.arms, tools):
        site_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, f"{arm}_tool0")
        err = np.linalg.norm(tool - env.data.site_xpos[site_id])
        print(f"{arm} FK/site check error: {err:.3e} m")
    print("Use the MuJoCo right-side Control sliders.")
    print("Either joint_1 slider controls the shared q1; the other side is mirrored to it.")
    print("q3/q4 sliders are Java-style actuator values; q4 is clamped each frame to q3 + offset.")
    print("Close the MuJoCo viewer window to stop.")
    viewer = mujoco.viewer.launch_passive(env.model, env.data, show_left_ui=True, show_right_ui=True)
    viewer.cam.distance = 2.8
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -25
    viewer.cam.lookat[:] = np.array([0.25, 0.0, 1.0])
    last_ctrl = env.data.ctrl[: env.action_dim].copy()
    try:
        while viewer.is_running():
            start = time.time()
            current_ctrl = env.data.ctrl[: env.action_dim].copy()
            right_q1 = env._arm_slice("right").start
            left_q1 = env._arm_slice("left").start
            right_delta = abs(current_ctrl[right_q1] - last_ctrl[right_q1])
            left_delta = abs(current_ctrl[left_q1] - last_ctrl[left_q1])
            if left_delta > right_delta and left_delta > 1e-9:
                q1_source = "left"
            elif right_delta > 1e-9:
                q1_source = "right"
            else:
                q1_source = "right"
            enforced_ctrl = env._enforce_ctrl_coupling(current_ctrl, q1_source=q1_source)
            env.data.ctrl[: env.action_dim] = enforced_ctrl
            last_ctrl = enforced_ctrl.copy()
            mujoco.mj_step(env.model, env.data)
            viewer.sync()
            time.sleep(max(0.0, 0.01 - (time.time() - start)))
    finally:
        viewer.close()
        env.close()


def parse_args():
    parser = argparse.ArgumentParser(description="AGV MuJoCo reach-and-avoid PPO")
    sub = parser.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    common.add_argument("--logdir", type=Path, default=Path("/tmp/agv_reach_logs"))
    common.add_argument("--seed", type=int, default=7)
    common.add_argument("--max-traj-len", type=int, default=160)
    common.add_argument("--obstacles", type=int, default=2)
    common.add_argument("--no-safety-shield", action="store_true")

    tr = sub.add_parser("train", parents=[common])
    tr.add_argument("--num-procs", type=int, default=15)
    tr.add_argument("--n-itr", type=int, default=5000)
    tr.add_argument("--eval-freq", type=int, default=50)
    tr.add_argument("--save-freq", type=int, default=100)
    tr.add_argument("--lr", type=float, default=3e-4)
    tr.add_argument("--gamma", type=float, default=0.98)
    tr.add_argument("--lam", type=float, default=0.95)
    tr.add_argument("--clip", type=float, default=0.2)
    tr.add_argument("--epochs", type=int, default=3)
    tr.add_argument("--minibatch-size", type=int, default=512)
    tr.add_argument("--std-dev", type=float, default=0.45)
    tr.add_argument("--entropy-coeff", type=float, default=0.002)
    tr.add_argument("--value-coeff", type=float, default=0.5)
    tr.add_argument("--max-grad-norm", type=float, default=0.5)
    tr.add_argument("--no-domain-randomization", action="store_true")
    tr.add_argument("--curriculum-iters", type=int, default=1200)
    tr.add_argument("--bc-steps", type=int, default=0)
    tr.add_argument("--bc-lr", type=float, default=1e-3)
    tr.add_argument("--bc-log-freq", type=int, default=100)
    tr.add_argument("--eval-episodes", type=int, default=0)
    tr.add_argument("--gate-episodes", type=int, default=0)
    tr.add_argument("--gate-success", type=float, default=0.85)
    tr.add_argument("--gate-collision", type=float, default=0.02)
    tr.add_argument("--no-ray-workers", action="store_true")

    ev = sub.add_parser("eval", parents=[common])
    ev.add_argument("--path", type=Path, default=None)
    ev.add_argument("--eval-domain-randomization", action="store_true")
    ev.add_argument("--view-speed", type=int, default=1)
    ev.add_argument("--no-realtime", action="store_true")

    sub.add_parser("inspect", parents=[common])
    return parser.parse_args()


def main():
    args = parse_args()
    if args.cmd == "train":
        train(args)
    elif args.cmd == "eval":
        eval_policy(args)
    elif args.cmd == "inspect":
        inspect_joints(args)


if __name__ == "__main__":
    main()
