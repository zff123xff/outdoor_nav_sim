# Resource provenance

World geometry, original depot resources originated in the local ROS2-Simulation/slam_sim workspace. Its MIT LICENSE and upstream model metadata remain included. Classic world files retain collision geometry and remove engine-specific systems. Depot decorative motor plugins were removed; visual and collision meshes were retained.

Agriculture geometry and textures come from Clearpath Robotics cpr_gazebo, pinned commit 509afbeec942c6aad376a5a76a24330f1c56dc17. BSD license and exact source are under models/cpr_agriculture. The reference world is the local robotdog_nav robot_scene/worlds/agriculture.world.

Navigation YAML originated as a local outdoor_nav configuration snapshot. The active shared configuration is simulation/config/nav.yaml, with a nonzero GPS origin for the hardware waypoint queue. Legacy robotdog_nav2.yaml remains for low-level profile regression tests. The real outdoor_nav workspace is not changed by these simulation edits.

Twist mux topics and priorities match outdoor_nav's outdoor_all_go2w.launch.py. Local Classic remote_stop_bridge only cancels navigation on nonzero manual input; twist_mux remains the sole arbiter. Robot model provenance is documented in ../sim_robot/SOURCE.md.
