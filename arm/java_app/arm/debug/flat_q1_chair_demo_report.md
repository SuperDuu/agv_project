# Flat Q1 Chair Demo Audit

label: `flatQ1ChairRight`
valid: `true`
frames: `389`
chair_distance_xy_mm: `112.54`
low_chair_height_mm: `94.65`
high_chair_height_mm: `102.45`
max_gripper_plane_tilt_deg_with_interpolation: `0.0689`
collision_validation: `true`

Self-check:
- The demo uses right-arm q1; q1 moves from +35 deg to -45 deg between the chairs.
- The parking pose is also on the horizontal-gripper manifold and is collision-free: `[0, 0, 80, -95, 0, 75]`.
- All right-arm poses are generated on the same horizontal-gripper manifold with A=q3+q4=-15 deg.
- The validator samples every frame and 7 interpolated points between frames; tolerance is +/-2 deg.
- The two chairs are intentionally separated mostly along Y.
- Self-collision validation and chair clearance both pass; the earlier `[0, 0, 20, -35, 0, 75]` parking pose was rejected because its TCP was too close to the torso.
