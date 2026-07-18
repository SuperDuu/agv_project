#!/usr/bin/env bash
# Huấn luyện AGV arm gắp vật thể trong MuJoCo, tránh vật cản phía trước.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEARNING_PY="/home/du/Desktop/LearningHumanoidWalking/.venv/bin/python"
LOGDIR="${LOGDIR:-/tmp/agv_reach_logs}"
N_ITR="${N_ITR:-5000}"
NUM_PROCS="${NUM_PROCS:-15}"
MAX_TRAJ_LEN="${MAX_TRAJ_LEN:-160}"
EVAL_FREQ="${EVAL_FREQ:-25}"
SEED="${SEED:-7}"

cd "$SCRIPT_DIR"
export PYTHONUNBUFFERED=1

echo "=========================================="
echo "  HUAN LUYEN AGV ARM - REACH/AVOID TASK   "
echo "=========================================="

rm -rf /tmp/mjcf-export/agv_reach

if [ -n "${PYTHON_BIN:-}" ]; then
  PY_CMD=("$PYTHON_BIN")
elif [ -x "$LEARNING_PY" ]; then
  PY_CMD=("$LEARNING_PY")
else
  PY_CMD=(uv run --with mujoco==3.4.0 --with torch==2.5.1 --with pyyaml --with numpy python)
fi

"${PY_CMD[@]}" -m rl_mujoco.ppo_reach train \
  --logdir "$LOGDIR" \
  --num-procs "$NUM_PROCS" \
  --n-itr "$N_ITR" \
  --eval-freq "$EVAL_FREQ" \
  --max-traj-len "$MAX_TRAJ_LEN" \
  --seed "$SEED"

echo "=========================================="
echo "  Huan luyen hoan tat."
echo "  Xem ket qua: ./view_custom.sh"
echo "=========================================="
