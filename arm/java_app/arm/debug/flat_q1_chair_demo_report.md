# Flat Q1 Chair Demo Audit

label: `flatQ1ChairRight`
valid: `true`
frames: `313`
chair_distance_xy_mm: `112.54`
low_chair_height_mm: `104.15`
high_chair_height_mm: `111.95`
max_gripper_plane_tilt_deg_with_interpolation: `0.5747`
collision_validation: `true`
object_jump_at_grip_mm: `0.0000`
object_jump_at_release_mm: `0.0000`
home_start_end: `true`
home_pause_after_first_release: `true`
event_delay_pattern: `pre=2000ms, post=3000ms`
motion_speed_source: `main angular speed slider`

Self-check:
- Dense flat path frames were reduced from 613 to 313 so the angular speed slider controls the actual motion more clearly.
- Non-event movement frames are marked pass-through; grip/release/home-pause frames still stop and wait.
- The gripper is open while not carrying the object, including the initial approach and after each release.
- The demo starts at HOME, transitions gradually to the flat parking pose, pauses at HOME after releasing on chair 2, then returns to chair 2.
- The right-arm flat-motion segments stay on the horizontal-gripper manifold with `A = q3 + q4 = -15 deg`.
