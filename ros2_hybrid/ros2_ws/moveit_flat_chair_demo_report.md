# MoveIt2 Flat Q1 Chair Demo Audit

- label: `moveitFlatQ1ChairRight`
- valid: `false`
- frames: `273`
- max_tilt_deg: `89.6866`
- flat_tilt_tolerance_deg: `2.0`
- max_joint_jump_deg: `9.99041`
- home_start_end: `true`
- OMPL_planner: `RRTConnect`
- OrientationConstraint: `Disabled`
- CollisionObjects: `2 Chairs (low/high)`

## Trajectory Summary
MoveIt2 computed a collision-free joint-space path around the chairs, but this run does not satisfy the horizontal gripper constraint. Keep the Java analytical flat-manifold trajectory for the live demo unless MoveIt2 is changed to use strict orientation/path constraints or a custom state constraint that enforces q2/q6 as functions of q5 and q3+q4.
