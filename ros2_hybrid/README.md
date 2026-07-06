# AGV ROS 2 Hybrid Workspace

This folder is the ROS 2 side of the Java/STM32 hybrid arm stack.

The Java Swing app can continue to run as the operator UI. ROS 2 will be added
as a motion-planning backend using MoveIt 2, with robot geometry and joint
limits kept in editable config files.

## Docker

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

## Current Goal

1. Generate a first URDF from editable robot parameters.
2. Validate the URDF in RViz with `robot_state_publisher`.
3. Add a bridge node so Java can request plans and receive joint trajectories.
4. Add MoveIt 2 configuration after the URDF matches the real robot geometry.
