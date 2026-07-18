#!/usr/bin/env bash
# Xem policy AGV arm trong MuJoCo viewer.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEARNING_PY="/home/du/Desktop/LearningHumanoidWalking/.venv/bin/python"
LOGDIR="${LOGDIR:-/tmp/agv_reach_logs}"
MAX_TRAJ_LEN="${MAX_TRAJ_LEN:-160}"
SEED="${SEED:-11}"
MODE="${MODE:-auto}"
VIEW_SPEED="${VIEW_SPEED:-4}"
VIEW_CURRICULUM="${VIEW_CURRICULUM:-1.0}"
VIEW_CHECKPOINT="${VIEW_CHECKPOINT:-latest}"
ACTOR_PATH="${ACTOR_PATH:-}"
NO_REALTIME="${NO_REALTIME:-0}"

cd "$SCRIPT_DIR"
export PYTHONUNBUFFERED=1

echo "=========================================="
echo "  XEM AGV ARM - MUJOCO VIEWER             "
echo "=========================================="
echo "  MODE=$MODE"
echo "  VIEW_SPEED=$VIEW_SPEED"
echo "  VIEW_CURRICULUM=$VIEW_CURRICULUM"
echo "  VIEW_CHECKPOINT=$VIEW_CHECKPOINT"
echo ""

rm -rf /tmp/mjcf-export/agv_reach

if [ -n "${PYTHON_BIN:-}" ]; then
  PY_CMD=("$PYTHON_BIN")
elif [ -x "$LEARNING_PY" ]; then
  PY_CMD=("$LEARNING_PY")
else
  PY_CMD=(uv run --with mujoco==3.4.0 --with torch==2.5.1 --with pyyaml --with numpy python)
fi

SELECTED_ACTOR="$ACTOR_PATH"
if [ -z "$SELECTED_ACTOR" ]; then
  case "$VIEW_CHECKPOINT" in
    latest)
      SELECTED_ACTOR="$(
        find "$LOGDIR" -path '*_agv_reach/actor_[0-9]*.pt' -type f -printf '%T@ %p\n' 2>/dev/null |
          sort -n |
          tail -1 |
          cut -d' ' -f2-
      )"
      ;;
    best)
      SELECTED_ACTOR=""
      ;;
    bc)
      SELECTED_ACTOR="$(
        find "$LOGDIR" -path '*_agv_reach/actor_bc.pt' -type f -printf '%T@ %p\n' 2>/dev/null |
          sort -n |
          tail -1 |
          cut -d' ' -f2-
      )"
      ;;
    final)
      SELECTED_ACTOR="$(
        find "$LOGDIR" -path '*_agv_reach/actor_final.pt' -type f -printf '%T@ %p\n' 2>/dev/null |
          sort -n |
          tail -1 |
          cut -d' ' -f2-
      )"
      ;;
    *)
      SELECTED_ACTOR="$(
        find "$LOGDIR" -path "*_agv_reach/$VIEW_CHECKPOINT" -type f -printf '%T@ %p\n' 2>/dev/null |
          sort -n |
          tail -1 |
          cut -d' ' -f2-
      )"
      ;;
  esac
fi
if [ -n "$SELECTED_ACTOR" ]; then
  echo "  ACTOR_PATH=$SELECTED_ACTOR"
fi

if [ "$MODE" = "inspect" ]; then
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach inspect \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED"
elif [ "$MODE" = "policy" ]; then
  EXTRA_ARGS=(--view-speed "$VIEW_SPEED" --curriculum "$VIEW_CURRICULUM")
  if [ "$NO_REALTIME" = "1" ]; then EXTRA_ARGS+=(--no-realtime); fi
  if [ -n "$SELECTED_ACTOR" ]; then EXTRA_ARGS+=(--path "$SELECTED_ACTOR"); fi
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach eval \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED" \
    "${EXTRA_ARGS[@]}"
elif [ -n "$SELECTED_ACTOR" ] || find "$LOGDIR" -name actor.pt -type f -print -quit 2>/dev/null | grep -q .; then
  EXTRA_ARGS=(--view-speed "$VIEW_SPEED" --curriculum "$VIEW_CURRICULUM")
  if [ "$NO_REALTIME" = "1" ]; then EXTRA_ARGS+=(--no-realtime); fi
  if [ -n "$SELECTED_ACTOR" ]; then EXTRA_ARGS+=(--path "$SELECTED_ACTOR"); fi
  "${PY_CMD[@]}" -m rl_mujoco.ppo_reach eval \
    --logdir "$LOGDIR" \
    --max-traj-len "$MAX_TRAJ_LEN" \
    --seed "$SEED" \
    "${EXTRA_ARGS[@]}"
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
