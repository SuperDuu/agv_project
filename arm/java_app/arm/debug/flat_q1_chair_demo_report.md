# Flat Q1 Chair Demo Audit

label: `flatQ1ChairRight`
valid: `true`
frames: `273`
chair_distance_xy_mm: `112.54`
low_chair_height_mm: `104.15`
high_chair_height_mm: `111.95`
max_gripper_plane_tilt_deg_with_interpolation: `0.5747`
home_start_end: `true`
home_pause_after_first_release: `true`
high_chair_home_route: `highPlace -> centerExit -> flatHome -> HOME -> flatHome -> centerExit -> highPlace`
event_delay_pattern: `pre=2000ms, post=3000ms`
motion_speed_source: `main angular speed slider`

Self-check:
- The demo starts at the app HOME pose, transitions to the flat parking pose, and ends back at HOME.
- After releasing on chair 2, the arm returns to HOME and waits before going back to pick from chair 2.
- The chair-2 to HOME route no longer passes through the chair-1 hover/exit waypoint.
- Non-event movement frames are marked pass-through; grip/release/home-pause frames still stop and wait.
- The gripper is open while not carrying the object, including the initial approach and after each release.
- The demo uses right-arm q1; q1 moves from +35 deg to -45 deg between the chairs.
- All right-arm poses are generated on the same horizontal-gripper manifold with A=q3+q4=-15 deg.
- The validator samples flat-motion frames and 7 interpolated points between frames; tolerance is +/-2 deg.
- The two chairs are intentionally separated mostly along Y.
