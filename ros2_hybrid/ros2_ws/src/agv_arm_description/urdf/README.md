URDF files in this package are generated from `config/agv_arm_geometry.yaml`.

Generate a URDF manually:

```bash
ros2 run agv_arm_description generate_agv_arm_urdf \
  --config src/agv_arm_description/config/agv_arm_geometry.yaml \
  --output /tmp/agv_dual_arm.urdf
```
