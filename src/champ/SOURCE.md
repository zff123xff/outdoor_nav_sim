# CHAMP source

Vendored from [rahgirrafi/unitree-go2-ros2](https://github.com/rahgirrafi/unitree-go2-ros2/tree/dfbc03947d9d9981a1cadc0a7d602adeb4386273/champ),
branch `ignition`, commit `dfbc03947d9d9981a1cadc0a7d602adeb4386273`.
Original framework: [chvmp/champ](https://github.com/chvmp/champ).

Only `champ`, `champ_msgs`, and `champ_base` are included. Copyright notices
and BSD licenses are retained. This project launches the gait controller only;
Gazebo, robot state publishing, sensors, and odometry stay in `slam_sim`.

Local adaptations:
- Use the node's ROS clock for the gait timer and phase calculation.
- Send immediate trajectory setpoints for continuous joint control.
- Stop requested velocity after the configurable `cmd_vel_timeout`.
- Hold the initial standing pose and ignore velocity commands while the Go2
  simulation's `/go2/recovering` flag is active; clear stored velocity on transitions.
- Keep the knee-orientation string alive, initialize flags, and reject invalid IK
  solutions rather than publishing uninitialized joint positions.
- Declare the core library's existing build dependencies in its package manifest.

Go2-specific joint/link mapping and gait settings are in `config/go2_champ.yaml`.
