"""Navigation can be restarted while Gazebo and PX4 EKF keep running."""
import os
import re
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def bringup(context):
    share = get_package_share_directory('south_lake_sim')
    ns = LaunchConfiguration('robot_namespace').perform(context).strip('/')
    if not ns or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        raise RuntimeError('robot_namespace must be one non-empty ROS name')
    source = LaunchConfiguration('params_file').perform(context)
    with open(source) as file:
        data = yaml.safe_load(file)
    frame_keys = {'global_frame', 'robot_base_frame', 'goal_output_frame'}

    def rewrite(obj):
        if isinstance(obj, dict):
            return {key: True if key == 'use_sim_time' else
                    ns+'/'+value if key in frame_keys and isinstance(value, str) else
                    '/'+ns+value if isinstance(value, str) and value.startswith('/livox/') else
                    rewrite(value) for key, value in obj.items()}
        return obj

    data = {'/'+ns+'/'+key: rewrite(value) for key, value in data.items()}
    bridge = data['/'+ns+'/nav2_input_bridge']['ros__parameters']
    bridge.update(local_odom_topic='mavros/odom', publish_odom_tf=True)
    temp = tempfile.NamedTemporaryFile(mode='w', prefix='south_lake_px4_nav_', suffix='.yaml', delete=False)
    yaml.safe_dump(data, temp, sort_keys=False)
    temp.close()
    params = [temp.name, {'use_sim_time': True}]
    return [
        Node(package='nav2_input_bridge', executable='input_bridge_node', name='nav2_input_bridge', namespace=ns, parameters=params, output='screen'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', namespace=ns,
             parameters=params, remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
        Node(package='nav2_straight_planner', executable='straight_planner_node', name='straight_planner', namespace=ns, parameters=params, output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', namespace=ns, parameters=params, output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config', 'go2_sim.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def profile_default(reference, current):
    return PythonExpression(["'", reference, "' if '", LaunchConfiguration('profile'), "' == 'robotdog_nav' else '", current, "'"])


def generate_launch_description():
    share = get_package_share_directory('south_lake_sim')
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robotdog_nav', choices=['robotdog_nav', 'current']),
        DeclareLaunchArgument('robot_namespace', default_value='robot'),
        DeclareLaunchArgument('params_file', default_value=profile_default(os.path.join(share, 'config', 'robotdog_nav2.yaml'), os.path.join(share, 'config', 'go2w_nav2.yaml'))),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=bringup),
    ])
