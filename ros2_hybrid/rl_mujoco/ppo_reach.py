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
    runs = [p for p in logdir.iterdir() if p.is_dir()] if logdir.exists() else []
    if not runs:
        raise FileNotFoundError(f"No training runs found under {logdir}")
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


def train(args):
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    run_dir = Path(args.logdir) / f"{datetime.now().strftime('%y-%m-%d-%H-%M-%S')}_agv_reach"
    run_dir.mkdir(parents=True, exist_ok=True)
    envs = [
        AgvReachEnv(args.geometry, seed=args.seed + i, max_steps=args.max_traj_len, obstacle_count=args.obstacles)
        for i in range(args.num_procs)
    ]
    obs_dim, action_dim = envs[0].obs_dim, envs[0].action_dim
    actor = Actor(obs_dim, action_dim, args.std_dev)
    critic = Critic(obs_dim)
    optimizer = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=args.lr, eps=1e-5)

    config = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    config["geometry"] = str(args.geometry)
    (run_dir / "config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    (run_dir / "model.xml").write_text(Path(envs[0].xml_path).read_text(encoding="utf-8"), encoding="utf-8")

    obs = np.stack([env.reset() for env in envs])
    best_success = 0.0
    print(f"Training AGV reach policy: logdir={run_dir}")
    for itr in range(1, args.n_itr + 1):
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

        obs_arr = np.asarray(obs_buf, dtype=np.float32).reshape(-1, obs_dim)
        act_arr = np.asarray(act_buf, dtype=np.float32).reshape(-1, action_dim)
        old_logp = np.asarray(logp_buf, dtype=np.float32).reshape(-1)
        rewards = np.asarray(rew_buf, dtype=np.float32)
        dones = np.asarray(done_buf, dtype=np.bool_)
        values = np.asarray(val_buf, dtype=np.float32)
        next_values = np.asarray(next_val_buf, dtype=np.float32)
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

        success_rate = float(np.mean(ep_success)) if ep_success else 0.0
        collision_rate = float(np.mean(ep_collision)) if ep_collision else 0.0
        mean_reward = float(rewards.sum(axis=0).mean())
        if itr % args.eval_freq == 0 or itr == 1:
            print(
                f"itr={itr:05d} reward={mean_reward:8.3f} "
                f"success={success_rate:5.2%} collision={collision_rate:5.2%}"
            )
        if success_rate >= best_success or itr % args.save_freq == 0:
            best_success = max(best_success, success_rate)
            save_checkpoint(run_dir / "actor.pt", actor, critic, args, obs_dim, action_dim)
        if itr % args.save_freq == 0:
            save_checkpoint(run_dir / f"actor_{itr}.pt", actor, critic, args, obs_dim, action_dim)

    save_checkpoint(run_dir / "actor.pt", actor, critic, args, obs_dim, action_dim)
    for env in envs:
        env.close()
    print(f"Training complete. Saved actor: {run_dir / 'actor.pt'}")


@torch.no_grad()
def eval_policy(args):
    run_dir = Path(args.path) if args.path else latest_run(Path(args.logdir))
    actor_path = run_dir if run_dir.is_file() else run_dir / "actor.pt"
    if not actor_path.exists():
        raise FileNotFoundError(f"Missing actor checkpoint: {actor_path}")
    actor = load_actor(actor_path)
    env = AgvReachEnv(args.geometry, seed=args.seed, max_steps=args.max_traj_len, obstacle_count=args.obstacles, render=True)
    obs = env.reset()
    print(f"Viewing policy: {actor_path}")
    print("Close the MuJoCo viewer window to stop.")
    while True:
        start = time.time()
        obs_t = torch.tensor(obs[None, :], dtype=torch.float32)
        action, _ = actor.act(obs_t, deterministic=True)
        obs, _, done, info = env.step(action.numpy()[0])
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
        time.sleep(max(0.0, env.dt - (time.time() - start)))
    env.close()


def inspect_joints(args):
    import mujoco
    import mujoco.viewer

    env = AgvReachEnv(args.geometry, seed=args.seed, max_steps=args.max_traj_len, obstacle_count=args.obstacles, render=True)
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
    print("Use the MuJoCo right-side Control sliders for right_joint_1_pos ... left_joint_6_pos.")
    print("Close the MuJoCo viewer window to stop.")
    mujoco.viewer.launch(env.model, env.data, show_left_ui=True, show_right_ui=True)
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

    tr = sub.add_parser("train", parents=[common])
    tr.add_argument("--num-procs", type=int, default=15)
    tr.add_argument("--n-itr", type=int, default=5000)
    tr.add_argument("--eval-freq", type=int, default=25)
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

    ev = sub.add_parser("eval", parents=[common])
    ev.add_argument("--path", type=Path, default=None)

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
