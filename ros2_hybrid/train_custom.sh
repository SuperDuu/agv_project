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
LR="${LR:-1e-4}"
CLIP="${CLIP:-0.1}"
BC_STEPS="${BC_STEPS:-0}"
BC_DATASET="${BC_DATASET:-/tmp/agv_reach_teacher/agv_reach_teacher.npz}"
BC_DATASET_STEPS="${BC_DATASET_STEPS:-1200}"
BC_BATCH_SIZE="${BC_BATCH_SIZE:-1024}"
POST_BC_STD="${POST_BC_STD:-0.08}"
MIN_STD="${MIN_STD:-0.03}"
MAX_STD="${MAX_STD:-0.12}"
ENTROPY_COEFF="${ENTROPY_COEFF:-0.0}"
BC_ANCHOR_COEFF="${BC_ANCHOR_COEFF:-2.0}"
BC_ANCHOR_ITERS="${BC_ANCHOR_ITERS:-2000}"
CRITIC_WARMUP_ITERS="${CRITIC_WARMUP_ITERS:-200}"
CURRICULUM_ITERS="${CURRICULUM_ITERS:-1200}"
ADAPTIVE_CURRICULUM="${ADAPTIVE_CURRICULUM:-1}"
CURRICULUM_SUCCESS="${CURRICULUM_SUCCESS:-0.65}"
CURRICULUM_STEP="${CURRICULUM_STEP:-0.025}"
CURRICULUM_EMA_ALPHA="${CURRICULUM_EMA_ALPHA:-0.3}"
CURRICULUM_MIN_EPISODES="${CURRICULUM_MIN_EPISODES:-8}"
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
echo "  BC_DATASET=$BC_DATASET"
echo "  LR=$LR CLIP=$CLIP"
echo "  POST_BC_STD=$POST_BC_STD MIN_STD=$MIN_STD MAX_STD=$MAX_STD ENTROPY_COEFF=$ENTROPY_COEFF"
echo "  BC_ANCHOR_COEFF=$BC_ANCHOR_COEFF BC_ANCHOR_ITERS=$BC_ANCHOR_ITERS CRITIC_WARMUP_ITERS=$CRITIC_WARMUP_ITERS"
echo "  ADAPTIVE_CURRICULUM=$ADAPTIVE_CURRICULUM SUCCESS=$CURRICULUM_SUCCESS STEP=$CURRICULUM_STEP EMA=$CURRICULUM_EMA_ALPHA"
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
if [ "$ADAPTIVE_CURRICULUM" = "0" ]; then
  EXTRA_ARGS+=(--no-adaptive-curriculum)
fi
if [ -f "$BC_DATASET" ]; then
  EXTRA_ARGS+=(--bc-dataset "$BC_DATASET" --bc-dataset-steps "$BC_DATASET_STEPS" --bc-batch-size "$BC_BATCH_SIZE")
else
  echo "  Chua co BC dataset, bo qua dataset warm-start."
  echo "  Tao dataset: ./make_teacher_dataset.sh"
fi

"${PY_CMD[@]}" -m rl_mujoco.ppo_reach train \
  --logdir "$LOGDIR" \
  --num-procs "$NUM_PROCS" \
  --n-itr "$N_ITR" \
  --eval-freq "$EVAL_FREQ" \
  --max-traj-len "$MAX_TRAJ_LEN" \
  --seed "$SEED" \
  --lr "$LR" \
  --clip "$CLIP" \
  --entropy-coeff "$ENTROPY_COEFF" \
  --bc-steps "$BC_STEPS" \
  --post-bc-std "$POST_BC_STD" \
  --min-std "$MIN_STD" \
  --max-std "$MAX_STD" \
  --bc-anchor-coeff "$BC_ANCHOR_COEFF" \
  --bc-anchor-iters "$BC_ANCHOR_ITERS" \
  --critic-warmup-iters "$CRITIC_WARMUP_ITERS" \
  --curriculum-iters "$CURRICULUM_ITERS" \
  --curriculum-success "$CURRICULUM_SUCCESS" \
  --curriculum-step "$CURRICULUM_STEP" \
  --curriculum-ema-alpha "$CURRICULUM_EMA_ALPHA" \
  --curriculum-min-episodes "$CURRICULUM_MIN_EPISODES" \
  --eval-episodes "$EVAL_EPISODES" \
  --gate-episodes "$GATE_EPISODES" \
  --gate-success "$GATE_SUCCESS" \
  --gate-collision "$GATE_COLLISION" \
  "${EXTRA_ARGS[@]}"

echo "=========================================="
echo "  Huan luyen hoan tat."
echo "  Xem ket qua: ./view_custom.sh"
echo "=========================================="
