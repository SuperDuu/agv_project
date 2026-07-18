#!/usr/bin/env bash
# Xem policy AGV arm trong MuJoCo viewer.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEARNING_PY="/home/du/Desktop/LearningHumanoidWalking/.venv/bin/python"
LOGDIR="${LOGDIR:-/tmp/agv_reach_logs}"
MAX_TRAJ_LEN="${MAX_TRAJ_LEN:-160}"
SEED="${SEED:-11}"
MODE="${MODE:-auto}"

cd "$SCRIPT_DIR"
export PYTHONUNBUFFERED=1

echo "=========================================="
echo "  XEM AGV ARM - MUJOCO VIEWER             "
echo "=========================================="
echo "  MODE=$MODE"
echo ""

rm -rf /tmp/mjcf-export/agv_reach

if [ -n "${PYTHON_BIN:-}" ]; then
  PY_CMD=("$PYTHON_BIN")
elif [ -x "$LEARNING_PY" ]; then
  PY_CMD=("$LEARNING_PY")
else
  PY_CMD=(uv run --with mujoco==3.4.0 --with torch==2.5.1 --with pyyaml --with numpy python)
fi

if [ "$MODE" = "inspect" ]; then
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach inspect \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED"
elif [ "$MODE" = "policy" ]; then
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach eval \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED"
elif find "$LOGDIR" -name actor.pt -type f -print -quit 2>/dev/null | grep -q .; then
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach eval \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED"
else
  echo "  Chua co checkpoint, mo joint inspector truoc."
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach inspect \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED"
fi

echo "=========================================="
echo "  Da dong viewer."
echo "=========================================="
