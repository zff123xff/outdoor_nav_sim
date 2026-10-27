"""Launch the ExecuteTask service server.

The node publishes to `waypoint_goals` (relative, resolves to
/{ns}/waypoint_goals) and offers the service on the ABSOLUTE path
`/agent/{agent_id}/execute_task` (matches the historical
/agent/uav1/execute_task callers), e.g. with agent_id=robot_a:

    /{ns}/waypoint_goals
    /agent/robot_a/execute_task

`agent_id` defaults to the node's `robot_namespace` so the JSON-level
agent_id check rejects tasks meant for a different robot. Override
`agent_id:=...` for the un-namespaced case (single robot on /).

Usage:
    ros2 launch execute_task_server execute_task.launch.py \
        robot_namespace:=robot_a agent_id:=robot_a
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Per-robot namespace. The node is pushed into this "
                    "namespace so its service and topic resolve under /{ns}/.",
    )
    agent_id_arg = DeclareLaunchArgument(
        "agent_id",
        default_value="",
        description="Agent ID used both for the service URL "
                    "(agent/{agent_id}/execute_task) and for the JSON-level "
                    "agent_id filter. Defaults to robot_namespace so the two "
                    "stay in sync; pass explicitly for the legacy single-robot "
                    "case (e.g. agent_id:=uav1).",
    )
    waypoint_frame_arg = DeclareLaunchArgument(
        "waypoint_frame",
        default_value="gps",
        description="Default waypoint frame (gps or local)",
    )

    ns = LaunchConfiguration("robot_namespace")

    execute_task_node = Node(
        package="execute_task_server",
        executable="execute_task_node",
        name="execute_task_node",
        namespace=ns,
        output="screen",
        parameters=[
            {
                # agent_id defaults to the namespace so the JSON check rejects
                # tasks meant for another robot. When robot_namespace is empty
                # (legacy single-robot usage), fall back to the literal default
                # below.
                "agent_id": LaunchConfiguration("agent_id"),
                "default_waypoint_frame": LaunchConfiguration("waypoint_frame"),
                "default_velocity": 1.0,
                "default_yaw": 0.0,
            },
        ],
    )

    return LaunchDescription([
        robot_namespace_arg,
        agent_id_arg,
        waypoint_frame_arg,
        execute_task_node,
    ])