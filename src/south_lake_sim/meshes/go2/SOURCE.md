These seven DAE files come from Unitree Robotics' official repository:

https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/go2_description/dae

Commit: `5994d4faef0a9cadd3287f8de0199a67eeb2a259`.
Retrieved on 2026-10-02. The original BSD 3-Clause license is in `LICENSE`.

The companion `urdf/go2/go2_body.xacro` is adapted from
`robots/go2_description/urdf/go2_description.urdf` at the same commit.
Changes: mesh package paths, `base` / `imu` frame names, a small positive
IMU inertia, and removal of extra material tags in each visual (URDF expects
one material per visual). Original body and leg inertias, joint limits and
collision shapes are retained.
