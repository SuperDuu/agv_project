# MoveIt2 Flat Q1 Chair Demo Audit

- label: `moveitFlatQ1ChairRight`
- valid: `true`
- frames: `273`
- max_tilt_pickplace_deg: `0.717553`
- max_tilt_all_deg: `75.2415`
- flat_tilt_tolerance_deg: `2`
- max_joint_jump_deg: `10.1458`
- home_start_end: `true`
- OMPL_planner: `RRTConnect`
- OrientationConstraint: `Pick/Place segments only`
- CollisionObjects: `2 Chairs (low/high)`

## Trajectory Summary
MoveIt2 successfully planned all 22 segments collision-free using OMPL RRTConnect. Orientation constraints (flat gripper) were applied only to pick/place approach segments; transit segments plan freely joint-to-joint. The pick/place frames satisfy the flat-gripper constraint with max_tilt_pickplace_deg < flat_tilt_tolerance_deg, confirming the gripper remains parallel to the ground during critical chair pick/place phases.
