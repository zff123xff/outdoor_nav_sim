# Pinned upstream sources

- PX4/PX4-ECL: https://github.com/PX4/PX4-ECL
  commit `46dd05a9159817035dab6acebc33f8a3da69d3a7` (official standalone ECL repository).
- PX4/Matrix: https://github.com/PX4/Matrix
  commit `f981cea2aebfc9cfd930dce73ba6d4d6681e99c1` (ECL's pinned dependency).

`vendor/ecl` and `vendor/matrix` preserve upstream source and licenses. EKF,
geographic projection, magnetic lookup and matrix headers are unchanged.
This is PX4's legacy 24-state ECL EKF, not a claim to match the unknown firmware
version on the real Pixhawk. ECL was archived in 2024; newer EKF2 changes live
in PX4-Autopilot. Matching a specific real FCU requires pinning its firmware.

Local CMake builds the standalone library without its network-fetching build
system. Local code only adapts ROS measurements and coordinates, produces
simulated sensors, and publishes estimates. It does not implement a new EKF.
The simulator truth is used only to generate GPS, magnetometer and pressure measurements
and for evaluation. The estimator cannot subscribe to robot truth pose, lidar,
FAST-LIO odometry, or commanded velocity.
