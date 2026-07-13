# Flat Q1 Chair Demo Audit

label: `flatQ1ChairRight`
valid: `true`
frames: `389`
chair_distance_xy_mm: `112.54`
low_chair_height_mm: `94.65`
high_chair_height_mm: `102.45`
max_gripper_plane_tilt_deg_with_interpolation: `0.0689`

Self-check:
- The demo uses right-arm q1; q1 moves from +35 deg to -45 deg between the two chairs.
- The two chairs are separated mostly along Y, with XY centers near `(-2.53, 57.49)` and `(2.48, -54.94)`.
- The right hand path is generated on the horizontal-gripper manifold with `A = q3 + q4 = -15 deg`.
- Validation samples every frame and 7 interpolated points between frames.
- Required gripper-plane tolerance is +/-2 deg; measured maximum is 0.0689 deg.
- Chair clearance was checked against the rendered chair boxes plus the existing 3 mm margin.
