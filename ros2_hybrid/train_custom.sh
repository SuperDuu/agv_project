#!/usr/bin/env bash
# Huấn luyện AGV arm gắp vật thể trong MuJoCo, tránh vật cản phía trước.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEARNING_PY="/home/du/Desktop/LearningHumanoidWalking/.venv/bin/python"
LOGDIR="${LOGDIR:-/tmp/agv_reach_logs}"
N_ITR="${N_ITR:-5000}"
NUM_PROCS="${NUM_PROCS:-15}"
MAX_TRAJ_LEN="${MAX_TRAJ_LEN:-160}"
EVAL_FREQ="${EVAL_FREQ:-50}"
SEED="${SEED:-7}"
BC_STEPS="${BC_STEPS:-0}"
CURRICULUM_ITERS="${CURRICULUM_ITERS:-1200}"
EVAL_EPISODES="${EVAL_EPISODES:-0}"
GATE_EPISODES="${GATE_EPISODES:-0}"
GATE_SUCCESS="${GATE_SUCCESS:-0.85}"
GATE_COLLISION="${GATE_COLLISION:-0.02}"
USE_RAY="${USE_RAY:-1}"
ULTRA_FAST_UNSAFE="${ULTRA_FAST_UNSAFE:-0}"
NO_DOMAIN_RANDOMIZATION="${NO_DOMAIN_RANDOMIZATION:-0}"
NO_SAFETY_SHIELD="${NO_SAFETY_SHIELD:-0}"

cd "$SCRIPT_DIR"
export PYTHONUNBUFFERED=1

echo "=========================================="
echo "  HUAN LUYEN AGV ARM - REACH/AVOID TASK   "
echo "=========================================="
echo "  BC_STEPS=$BC_STEPS EVAL_EPISODES=$EVAL_EPISODES GATE_EPISODES=$GATE_EPISODES"
echo "  NUM_PROCS=$NUM_PROCS USE_RAY=$USE_RAY"
echo "  ULTRA_FAST_UNSAFE=$ULTRA_FAST_UNSAFE"

rm -rf /tmp/mjcf-export/agv_reach

if [ -n "${PYTHON_BIN:-}" ]; then
  PY_CMD=("$PYTHON_BIN")
elif [ -x "$LEARNING_PY" ]; then
  PY_CMD=("$LEARNING_PY")
else
  PY_CMD=(uv run --with mujoco==3.4.0 --with torch==2.5.1 --with pyyaml --with numpy python)
fi

EXTRA_ARGS=()
if [ "$ULTRA_FAST_UNSAFE" = "1" ]; then
  EXTRA_ARGS+=(--no-domain-randomization --no-safety-shield)
elif [ "$NO_DOMAIN_RANDOMIZATION" = "1" ]; then
  EXTRA_ARGS+=(--no-domain-randomization)
fi
if [ "$NO_SAFETY_SHIELD" = "1" ]; then
  EXTRA_ARGS+=(--no-safety-shield)
fi
if [ "$USE_RAY" = "0" ]; then
  EXTRA_ARGS+=(--no-ray-workers)
fi

"${PY_CMD[@]}" -m rl_mujoco.ppo_reach train \
  --logdir "$LOGDIR" \
  --num-procs "$NUM_PROCS" \
  --n-itr "$N_ITR" \
  --eval-freq "$EVAL_FREQ" \
  --max-traj-len "$MAX_TRAJ_LEN" \
  --seed "$SEED" \
  --bc-steps "$BC_STEPS" \
  --curriculum-iters "$CURRICULUM_ITERS" \
  --eval-episodes "$EVAL_EPISODES" \
  --gate-episodes "$GATE_EPISODES" \
  --gate-success "$GATE_SUCCESS" \
  --gate-collision "$GATE_COLLISION" \
  "${EXTRA_ARGS[@]}"

echo "=========================================="
echo "  Huan luyen hoan tat."
echo "  Xem ket qua: ./view_custom.sh"
echo "=========================================="
