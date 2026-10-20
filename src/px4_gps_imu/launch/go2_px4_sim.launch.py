"""Gazebo Classic Go2-W, CHAMP / wheel control and original PX4 ECL."""
import math
import os
from pathlib import Path
import re
import tempfile
from xml.dom import minidom

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction, Shutdown, GroupAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def after_success(actions):
    def callback(event, context):
        if event.returncode != 0:
            return [Shutdown(reason='Classic robot spawn/controller activation failed')]
        return actions if isinstance(actions, list) else [actions]
    return callback


def profile_default(reference, current):
    return PythonExpression(["'", reference, "' if '", LaunchConfiguration('profile'),
                             "' == 'robotdog_nav' else '", current, "'"])


def bringup(context):
    share = get_package_share_directory('south_lake_sim')
    model_share = get_package_share_directory('go2w_sim')
    ekf_share = get_package_share_directory('px4_gps_imu')
    lidar_share = get_package_share_directory('mid360_simulation')
    get_package_share_directory('gazebo_ros2_control')
    def value(key):
        return LaunchConfiguration(key).perform(context)
    ns = value('robot_namespace').strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        raise RuntimeError('robot_namespace must be one non-empty ROS name')
    hybrid = value('locomotion_mode') == 'hybrid'
    with open(value('params_file')) as stream:
        nav = yaml.safe_load(stream)
    bridge = nav['nav2_input_bridge']['ros__parameters']
    reference = {key: bridge[key] for key in ('reference_latitude', 'reference_longitude', 'reference_altitude')}
    with open(value('spawn_config')) as stream:
        config = yaml.safe_load(stream)
    def number(raw, name, positive=False):
        if isinstance(raw, bool):
            raise RuntimeError(name+' must be numeric, not boolean')
        raw = float(raw)
        if not math.isfinite(raw) or positive and raw <= 0:
            raise RuntimeError(name+' must be finite'+(' and positive' if positive else ''))
        return raw
    spawn = {}
    for axis in ('x', 'y', 'z', 'yaw'):
        raw = value('spawn_'+axis).strip() or config['spawn'][axis]
        spawn[axis] = .03417389 if axis == 'z' and raw == 'auto' else number(raw, 'spawn.'+axis)
    speed = number(value('gait_max_speed').strip() or config['drive']['max_linear_speed'], 'drive.max_linear_speed', True)
    turn = number(value('gait_max_yaw_rate').strip() or config['drive']['max_angular_speed'], 'drive.max_angular_speed', True)
    mappings = {'robot_namespace': ns, 'leg_command_interface': 'effort' if hybrid else 'position'}
    if value('sensor_config'):
        with open(value('sensor_config')) as stream:
            sensors = yaml.safe_load(stream)
        if not isinstance(sensors, dict):
            raise RuntimeError('sensor_config must contain xacro argument mappings')
        mappings.update({key: str(raw) for key, raw in sensors.items()})
    description = xacro.process_file(os.path.join(model_share, 'urdf', 'go2w.xacro'), mappings=mappings).toxml()
    with open(os.path.join(model_share, 'config', 'classic_hybrid_controllers.yaml' if hybrid else 'controllers.yaml')) as stream:
        control = yaml.safe_load(stream)
    if not hybrid:
        wheel = control['diff_drive_controller']['ros__parameters']
        wheel.update(base_frame_id=ns+'/base_link', odom_frame_id=ns+'/odom')
        for axis, limit in [('linear.x', speed), ('angular.z', turn)]:
            wheel[axis+'.max_velocity'] = limit
            wheel[axis+'.min_velocity'] = -limit
    controllers = tempfile.NamedTemporaryFile(mode='w', prefix='go2w_classic_controllers_', suffix='.yaml', delete=False)
    yaml.safe_dump(control, controllers, sort_keys=False)
    controllers.close()
    description = description.replace('__GO2W_CONTROLLERS_YAML__', controllers.name).replace('__GO2W_CMD_VEL__', '/'+ns+'/cmd_vel')
    sensor_xml = '<gazebo><plugin filename="libpx4_classic_sensors.so" name="px4_sim_sensors"><ros><namespace>/'+ns+'</namespace></ros><robot_namespace>'+ns+'</robot_namespace>'
    for key, raw in reference.items():
        sensor_xml += '<'+key+'>'+str(number(raw, key))+'</'+key+'>'
    for key in ('gps_position_stddev', 'gps_velocity_stddev', 'gps_update_rate'):
        raw = number(value(key), key)
        if key == 'gps_update_rate' and not 0 < raw <= 200 or key != 'gps_update_rate' and not 0 <= raw < 100:
            raise RuntimeError('Invalid '+key)
        sensor_xml += '<'+key+'>'+str(raw)+'</'+key+'>'
    description = description.replace('</robot>', sensor_xml+'</plugin></gazebo></robot>')
    doc = minidom.parseString(description)
    for mesh in doc.getElementsByTagName('mesh'):
        filename = mesh.getAttribute('filename')
        for pkg, directory in [('go2w_sim', model_share), ('south_lake_sim', share)]:
            if filename.startswith('package://'+pkg+'/'):
                mesh.setAttribute('filename', (Path(directory)/filename.split(pkg+'/', 1)[1]).as_uri())
    description = doc.toxml()
    robot_file = tempfile.NamedTemporaryFile(mode='w', prefix='go2w_classic_', suffix='.urdf', delete=False)
    robot_file.write(description)
    robot_file.close()
    world = value('world')
    world_path = world if os.path.isabs(world) else os.path.join(share, 'worlds', world)
    if not Path(world_path).is_file():
        raise RuntimeError('Classic world not found: '+world_path)
    spawn_node = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'go2w', '-file', robot_file.name, '-timeout', '120',
                   '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(spawn['z']), '-Y', str(spawn['yaw'])])
    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--inactive', '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '120', '--service-call-timeout', '120'])
    joints = spawner('joint_state_broadcaster')
    legs = spawner('leg_trajectory_controller' if hybrid else 'leg_position_controller')
    wheels = spawner('wheel_velocity_controller' if hybrid else 'diff_drive_controller')
    ekf = Node(package='px4_gps_imu', executable='px4_ekf_node', namespace=ns,
        parameters=[dict(reference, use_sim_time=True, imu_xyz=[-.02557, 0., .47814611])], output='screen')
    if hybrid:
        control_actions = [
            Node(package='champ_base', executable='quadruped_controller_node', name='go2w_champ_controller', namespace=ns,
                parameters=[os.path.join(model_share, 'config', 'hybrid_champ.yaml'),
                    {'use_sim_time': True, 'gazebo': True, 'publish_joint_control': True,
                     'publish_joint_states': False, 'publish_foot_contacts': False,
                     'gait.max_angular_velocity_z': turn, 'urdf': description,
                     'joint_controller_topic': '/leg_trajectory_controller/joint_trajectory'}],
                remappings=[('cmd_vel/smooth', 'cmd_vel_legs')],
                on_exit=[Shutdown(reason='Go2-W CHAMP exited')], output='screen'),
            Node(package='go2w_sim', executable='hybrid_motion_controller.py', namespace=ns,
                parameters=[os.path.join(model_share, 'config', 'hybrid_drive.yaml'),
                    {'use_sim_time': True, 'max_linear_speed': speed, 'max_angular_speed': turn}],
                remappings=[('wheel_commands', '/wheel_velocity_controller/commands')],
                on_exit=[Shutdown(reason='Go2-W hybrid drive exited')], output='screen')]
    else:
        control_actions = [Node(package='go2w_sim', executable='stand_initializer.py', parameters=[{'use_sim_time': True}], output='screen')]
    start = Node(package='go2w_sim', executable='classic_start_controllers.py',
        parameters=[{'controllers': ['joint_state_broadcaster', 'leg_trajectory_controller' if hybrid else 'leg_position_controller',
                                    'wheel_velocity_controller' if hybrid else 'diff_drive_controller']}], output='screen')
    control_actions.append(start)
    plugins = os.pathsep.join([str(Path(ekf_share).parent.parent/'lib'), str(Path(lidar_share).parent.parent/'lib'), os.environ.get('GAZEBO_PLUGIN_PATH', '')])
    models = os.pathsep.join([os.path.join(share, 'models'), str(Path(share).parent), os.environ.get('GAZEBO_MODEL_PATH', '')])
    return [
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugins),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', models),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', os.environ.get('GAZEBO_MODEL_DATABASE_URI', '')),
        GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world_path, 'verbose': 'true', 'pause': 'true', 'profile': '', 'params_file': '', 'server_required': 'true'}.items())]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': True, 'frame_prefix': ns+'/'}], output='screen'),
        Node(package='twist_mux', executable='twist_mux', namespace=ns,
            parameters=[os.path.join(share, 'config', 'sim_twist_mux.yaml'), {'use_sim_time': True}], remappings=[('cmd_vel_out', 'cmd_vel')], output='screen'),
        Node(package='south_lake_sim', executable='remote_stop_bridge.py', namespace=ns, parameters=[{'use_sim_time': True}], output='screen'),
        TimerAction(period=2., actions=[spawn_node]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_node, on_exit=after_success(joints))),
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=after_success(legs))),
        RegisterEventHandler(OnProcessExit(target_action=legs, on_exit=after_success(wheels))),
        RegisterEventHandler(OnProcessExit(target_action=wheels, on_exit=after_success(control_actions))),
        RegisterEventHandler(OnProcessExit(target_action=start, on_exit=after_success(TimerAction(period=5., actions=[ekf])))),
        Node(package='tf2_ros', executable='static_transform_publisher',
            arguments=['--frame-id', ns+'/map', '--child-frame-id', ns+'/odom'], parameters=[{'use_sim_time': True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(ekf_share, 'launch', 'go2_px4_nav.launch.py')),
            launch_arguments={'profile': value('profile'), 'robot_namespace': ns, 'params_file': value('params_file'), 'rviz': value('rviz')}.items(),
            condition=IfCondition(LaunchConfiguration('navigation'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('south_lake_sim')
    config = lambda name: os.path.join(share, 'config', name)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robotdog_nav', choices=['robotdog_nav', 'current']),
        DeclareLaunchArgument('robot_namespace', default_value='robot'),
        DeclareLaunchArgument('locomotion_mode', default_value=profile_default('hybrid', 'wheels'), choices=['hybrid', 'wheels']),
        DeclareLaunchArgument('world', default_value=profile_default('agriculture.world', 'depot.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=profile_default(config('robotdog_nav2.yaml'), config('go2w_nav2.yaml'))),
        DeclareLaunchArgument('spawn_config', default_value=profile_default(config('robotdog_spawn.yaml'), config('sim_spawn.yaml'))),
        DeclareLaunchArgument('sensor_config', default_value=profile_default(config('robotdog_sensors.yaml'), '')),
        *[DeclareLaunchArgument('spawn_'+axis, default_value='') for axis in ('x', 'y', 'z', 'yaw')],
        DeclareLaunchArgument('gait_max_speed', default_value=''),
        DeclareLaunchArgument('gait_max_yaw_rate', default_value=''),
        DeclareLaunchArgument('gps_update_rate', default_value=profile_default('30', '10')),
        DeclareLaunchArgument('gps_position_stddev', default_value=profile_default('0.0002', '0.15')),
        DeclareLaunchArgument('gps_velocity_stddev', default_value='0.03'),
        OpaqueFunction(function=bringup),
    ])
