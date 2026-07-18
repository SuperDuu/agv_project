#!/usr/bin/env bash
# Generate planner-teacher observations/actions for PPO BC warm-start.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEARNING_PY="/home/du/Desktop/LearningHumanoidWalking/.venv/bin/python"
OUT="${OUT:-/tmp/agv_reach_teacher/agv_reach_teacher.npz}"
EPISODES="${EPISODES:-800}"
MAX_TRAJ_LEN="${MAX_TRAJ_LEN:-160}"
OBSTACLES="${OBSTACLES:-2}"
SEED="${SEED:-7007}"
PLANNER_NODES="${PLANNER_NODES:-700}"
MIN_CURRICULUM="${MIN_CURRICULUM:-0.0}"
MAX_CURRICULUM="${MAX_CURRICULUM:-1.0}"
CURRICULUM="${CURRICULUM:--1.0}"
CURRICULUM_POWER="${CURRICULUM_POWER:-0.5}"
DOMAIN_RANDOMIZATION="${DOMAIN_RANDOMIZATION:-0}"

cd "$SCRIPT_DIR"
export PYTHONUNBUFFERED=1

if [ -n "${PYTHON_BIN:-}" ]; then
  PY_CMD=("$PYTHON_BIN")
elif [ -x "$LEARNING_PY" ]; then
  PY_CMD=("$LEARNING_PY")
else
  PY_CMD=(uv run --with mujoco==3.4.0 --with torch==2.5.1 --with pyyaml --with numpy python)
fi

EXTRA_ARGS=()
if [ "$DOMAIN_RANDOMIZATION" = "1" ]; then
  EXTRA_ARGS+=(--domain-randomization)
fi

echo "=========================================="
echo "  SINH TEACHER DATASET CHO AGV PPO        "
echo "=========================================="
echo "  OUT=$OUT"
echo "  EPISODES=$EPISODES MAX_TRAJ_LEN=$MAX_TRAJ_LEN"
echo "  CURRICULUM=$CURRICULUM RANGE=[$MIN_CURRICULUM,$MAX_CURRICULUM] POWER=$CURRICULUM_POWER"

"${PY_CMD[@]}" -m rl_mujoco.make_teacher_dataset \
  --out "$OUT" \
  --episodes "$EPISODES" \
  --max-traj-len "$MAX_TRAJ_LEN" \
  --obstacles "$OBSTACLES" \
  --seed "$SEED" \
  --planner-nodes "$PLANNER_NODES" \
  --curriculum "$CURRICULUM" \
  --min-curriculum "$MIN_CURRICULUM" \
  --max-curriculum "$MAX_CURRICULUM" \
  --curriculum-power "$CURRICULUM_POWER" \
  "${EXTRA_ARGS[@]}"

echo "=========================================="
echo "  Xong. Train se tu dung dataset neu file ton tai:"
echo "  BC_DATASET=$OUT ./train_custom.sh"
echo "=========================================="
