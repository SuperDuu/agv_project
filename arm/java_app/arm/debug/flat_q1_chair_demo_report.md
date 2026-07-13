# Flat Q1 Chair Demo Audit

label: `flatQ1ChairRight`
valid: `true`
frames: `613`
chair_distance_xy_mm: `112.54`
low_chair_height_mm: `104.15`
high_chair_height_mm: `111.95`
max_gripper_plane_tilt_deg_with_interpolation: `0.1530`
collision_validation: `true`
object_jump_at_grip_mm: `0.0000`
object_jump_at_release_mm: `0.0000`
home_start_end: `true`
home_pause_after_first_release: `true`
event_delay_pattern: `pre=2000ms, post=3000ms`

Self-check:
- The demo starts at the app HOME pose `[0, 0, 20, -35, 0, 0]`, transitions gradually to the flat parking pose, and ends back at HOME.
- After releasing on chair 2, the arm returns to HOME and waits before going back to pick from chair 2.
- The demo uses right-arm q1; q1 moves from +35 deg to -45 deg between the chairs.
- The object center on each chair is aligned with the TCP at grip/release, so the object does not jump upward when gripped.
- The wrist direction is changed at the low-chair side before q1 sweeps to the high chair, avoiding the high-chair box while preserving the flat gripper constraint.
- All right-arm flat-motion poses are generated on the same horizontal-gripper manifold with `A = q3 + q4 = -15 deg`.
- Validation samples flat-motion frames and 7 interpolated points between frames; tolerance is +/-2 deg.
- Self-collision validation and chair clearance both pass.
