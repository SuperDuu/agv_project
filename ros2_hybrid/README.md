# AGV ROS 2 Hybrid Workspace

This folder is the ROS 2 side of the Java/STM32 hybrid arm stack.

The Java Swing app can continue to run as the operator UI. ROS 2 will be added
as a motion-planning backend using MoveIt 2, with robot geometry and joint
limits kept in editable config files.

## Docker

On Windows, install and start Docker Desktop before running the commands below.
Docker Desktop should use the WSL 2 backend. After installation, restart
PowerShell and verify:

```powershell
docker --version
docker compose version
```

If PowerShell says `docker` is not recognized, Docker Desktop is not installed,
is not running, or its CLI path has not been added to the current terminal
session.

Build the container:

```bash
docker compose build
```

Open a ROS 2 shell:

```bash
docker compose run --rm ros2
```

Inside the container, build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Editable Robot Parameters

Robot dimensions and joint limits live in:

```text
ros2_ws/src/agv_arm_description/config/agv_arm_geometry.yaml
```

The current values are placeholders copied from the Java prototype. Before
using real hardware planning, replace them with measured values:

- `links.L0..L7`: link lengths in millimeters.
- `base.shoulder_z`, `base.shoulder_y_offset`: torso/shoulder placement in millimeters.
- `joint_limits.right/left`: joint limits in degrees plus velocity/effort limits.

Generate a URDF inside the container:

```bash
source install/setup.bash
ros2 run agv_arm_description generate_agv_arm_urdf \
  --config src/agv_arm_description/config/agv_arm_geometry.yaml \
  --output /tmp/agv_dual_arm.urdf
```

Display the generated robot model:

```bash
ros2 launch agv_arm_description display.launch.py
```

## Java Bridge

Start the Java/ROS 2 UDP bridge:

```bash
ros2 launch agv_arm_bridge java_udp_bridge.launch.py
```

Java can send a UDP JSON packet to port `5010`:

```json
{
  "type": "plan_pose",
  "request_id": "demo-001",
  "arm": "right",
  "target": { "x": 120.0, "y": 0.0, "z": 20.0 },
  "reply_host": "127.0.0.1",
  "reply_port": 5011
}
```

The bridge publishes the request to:

```text
/agv_arm/plan_requests
```

The next integration step is to add a MoveIt 2 planner node that subscribes to
that topic, plans a `JointTrajectory`, and sends the joint list back to Java or
directly to a `ros2_control` hardware interface.

## Current Goal

1. Generate a first URDF from editable robot parameters.
2. Validate the URDF in RViz with `robot_state_publisher`.
3. Add a bridge node so Java can request plans and receive joint trajectories.
4. Add MoveIt 2 configuration after the URDF matches the real robot geometry.
