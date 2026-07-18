#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-agv_ros2_rviz}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
IMAGE_NAME="${IMAGE_NAME:-agv-ros2-hybrid:jazzy}"
FORCE_BUILD="${FORCE_BUILD:-0}"

cd "$SCRIPT_DIR"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command not found" >&2
  exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set; open a graphical Ubuntu session before running RViz." >&2
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +SI:localuser:root >/dev/null
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
  docker rm "$CONTAINER_NAME" >/dev/null
fi

if [ "$FORCE_BUILD" = "1" ] || ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  docker compose build
else
  echo "Using existing Docker image: $IMAGE_NAME"
fi

docker compose run -d \
  --name "$CONTAINER_NAME" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
  -e GALLIUM_DRIVER=llvmpipe \
  -e QT_XCB_GL_INTEGRATION=xcb_egl \
  ros2 bash -lc "source /opt/ros/jazzy/setup.bash; colcon build --symlink-install --packages-select agv_arm_moveit agv_arm_bridge agv_arm_description; source install/setup.bash; ros2 launch agv_arm_description display.launch.py"

echo "RViz container started: $CONTAINER_NAME"
echo "Follow logs: docker logs -f $CONTAINER_NAME"
