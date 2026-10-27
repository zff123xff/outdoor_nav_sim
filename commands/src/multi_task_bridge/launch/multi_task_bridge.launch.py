from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression

def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id")
    drone_name = LaunchConfiguration("drone_name")
    # drone_name = PythonExpression(["'drone_'+str(", drone_id, ")"]),
    flight_alt_m = LaunchConfiguration("flight_alt_m")
    swath_width_m = LaunchConfiguration("swath_width_m")

    params_file = PathJoinSubstitution([
        FindPackageShare("multi_task_bridge"),
        "config",
        "params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="1"),
        DeclareLaunchArgument("drone_name", default_value="uav1"),
        DeclareLaunchArgument("flight_alt_m", default_value="5.0"),
        DeclareLaunchArgument("swath_width_m", default_value="10.0"),
        Node(
            package="multi_task_bridge",
            executable="multi_task_bridge",
            name=PythonExpression(["'multi_task_bridge_'+str(", drone_id, ")"]),
            output="screen",
            parameters=[
                params_file,
                {
                    "drone_id": drone_id,
                    "drone_name": drone_name,
                    "flight_alt_m": flight_alt_m,
                    "swath_width_m": swath_width_m,
                },
            ],
        ),
    ])
