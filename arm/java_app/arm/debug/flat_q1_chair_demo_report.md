# Flat Q1 Chair Demo Audit

label: `flatQ1ChairRight`
valid: `true`
frames: `517`
chair_distance_xy_mm: `112.54`
low_chair_height_mm: `104.15`
high_chair_height_mm: `111.95`
max_gripper_plane_tilt_deg_with_interpolation: `0.1530`
collision_validation: `true`
object_jump_at_grip_mm: `0.0000`
object_jump_at_release_mm: `0.0000`

Self-check:
- The demo uses right-arm q1; q1 moves from +35 deg to -45 deg between the chairs.
- The parking pose is on the horizontal-gripper manifold and is collision-free: `[0, 0, 80, -95, 0, 75]`.
- The object center on each chair is aligned with the TCP at grip/release, so the object no longer jumps upward when gripped.
- The wrist direction is changed at the low-chair side before q1 sweeps to the high chair, avoiding the high-chair box while preserving the flat gripper constraint.
- All right-arm poses are generated on the same horizontal-gripper manifold with `A = q3 + q4 = -15 deg`.
- Validation samples every frame and 7 interpolated points between frames; tolerance is +/-2 deg.
- Self-collision validation and chair clearance both pass.
