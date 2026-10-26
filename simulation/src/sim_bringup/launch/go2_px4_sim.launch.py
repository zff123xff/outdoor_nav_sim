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


def namespace_rviz_config(source, namespace):
    """Keep RViz frames, model prefix, displays and goal tool on one robot."""
    with open(source) as stream:
        config = yaml.safe_load(stream)

    def rewrite(value, key=None):
        if isinstance(value, dict):
            return {k: rewrite(v, k) for k, v in value.items()}
        if isinstance(value, list):
            return [rewrite(v) for v in value]
        if isinstance(value, str):
            if key == 'TF Prefix' and value == 'robot':
                return namespace
            if value.startswith('/robot/'):
                return '/'+namespace+value[len('/robot'):]
            if value.startswith('robot/'):
                return namespace+value[len('robot'):]
        return value

    config = rewrite(config)
    with tempfile.NamedTemporaryFile(mode='w', prefix='south_lake_rviz_'+namespace+'_',
                                     suffix='.rviz', delete=False) as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
        return stream.name



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
    bringup_share = get_package_share_directory('simulation_bringup')
    lidar_share = get_package_share_directory('mid360_simulation')
    get_package_share_directory('gazebo_ros2_control')
    def value(key):
        return LaunchConfiguration(key).perform(context)
    with open(value('params_file')) as stream:
        nav = yaml.safe_load(stream)
    ns = value('robot_namespace') or nav.get('launch_config', {}).get('ros__parameters', {}).get('robot_namespace', 'robot_1')
    if not isinstance(ns, str):
        raise RuntimeError('robot_namespace must be a string')
    ns = ns.strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        raise RuntimeError('robot_namespace must be one non-empty ROS name')
    hybrid = value('locomotion_mode') == 'hybrid'
    zny = value('interface') == 'zny'
    if zny and not hybrid:
        raise RuntimeError('interface:=zny requires locomotion_mode:=hybrid to support lateral velocity')
    if value('navigation').lower() == 'true':
        raise RuntimeError('navigation is no longer started here; use outdoor_nav/launch_hil.sh')
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
    if zny:
        speed, turn = min(speed, 1.5), min(turn, 1.0)
    mappings = {'robot_namespace': ns, 'leg_command_interface': 'effort' if hybrid else 'position'}
    if value('sensor_config'):
        with open(value('sensor_config')) as stream:
            sensors = yaml.safe_load(stream)
        if not isinstance(sensors, dict):
            raise RuntimeError('sensor_config must contain xacro argument mappings')
        mappings.update({key: str(raw) for key, raw in sensors.items()})
    if zny:
        # Body origin is 0.43582611 m above navigation base_link.
        mappings.update(zny_interface='true', lidar_xyz='0.18 0 0.09417389',
                        lidar_rpy='0 0.3141592653589793 0', lidar_publish_rate='10',
                        lidar_sensor_update_rate='100')
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
        parameters=[dict(reference, use_sim_time=True, imu_xyz=[-.02557, 0., .47814611],
                         zny_interface=zny, odom_publish_rate=100.0 if zny else 50.0)],
        # External navigation owns /odom through its QoS relay.
        remappings=[('odom', 'simulation/ekf_odom')], output='screen')
    if hybrid:
        control_actions = [
            Node(package='champ_base', executable='quadruped_controller_node', name='go2w_champ_controller', namespace=ns,
                parameters=[os.path.join(model_share, 'config', 'hybrid_champ.yaml'),
                    {'use_sim_time': True, 'gazebo': True, 'publish_joint_control': True,
                     'publish_joint_states': False, 'publish_foot_contacts': False,
                     'gait.max_angular_velocity_z': turn, 'urdf': description,
                     **({'gait.max_linear_velocity_y': .6} if zny else {}),
                     'joint_controller_topic': '/leg_trajectory_controller/joint_trajectory'}],
                remappings=[('cmd_vel/smooth', 'cmd_vel_legs')],
                on_exit=[Shutdown(reason='Go2-W CHAMP exited')], output='screen'),
            Node(package='go2w_sim', executable='hybrid_motion_controller.py', namespace=ns,
                parameters=[os.path.join(model_share, 'config', 'hybrid_drive.yaml'),
                    {'use_sim_time': True, 'max_linear_speed': speed, 'max_angular_speed': turn,
                     **({'max_lateral_speed': .6} if zny else {})}],
                remappings=[('wheel_commands', '/wheel_velocity_controller/commands')],
                on_exit=[Shutdown(reason='Go2-W hybrid drive exited')], output='screen')]
    else:
        control_actions = [Node(package='go2w_sim', executable='stand_initializer.py', parameters=[{'use_sim_time': True}], output='screen')]
    start = Node(package='go2w_sim', executable='classic_start_controllers.py',
        parameters=[{'controllers': ['joint_state_broadcaster', 'leg_trajectory_controller' if hybrid else 'leg_position_controller',
                                    'wheel_velocity_controller' if hybrid else 'diff_drive_controller']}], output='screen')
    control_actions.append(start)
    plugins = os.pathsep.join([str(Path(bringup_share).parent.parent/'lib'), str(Path(lidar_share).parent.parent/'lib'), os.environ.get('GAZEBO_PLUGIN_PATH', '')])
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
            parameters=[{'robot_description': description, 'use_sim_time': True, 'frame_prefix': ns+'/'}],
            remappings=[('/tf', '/'+ns+'/simulation/tf'), ('/tf_static', '/'+ns+'/simulation/tf_static')] if zny else [], output='screen'),
        Node(package='south_lake_sim', executable='remote_stop_bridge.py', namespace=ns, parameters=[{'use_sim_time': True}],
             condition=IfCondition(PythonExpression(['not ', str(zny)])), output='screen'),
        TimerAction(period=2., actions=[spawn_node]),
        RegisterEventHandler(OnProcessExit(target_action=spawn_node, on_exit=after_success(joints))),
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=after_success(legs))),
        RegisterEventHandler(OnProcessExit(target_action=legs, on_exit=after_success(wheels))),
        RegisterEventHandler(OnProcessExit(target_action=wheels, on_exit=after_success(control_actions))),
        RegisterEventHandler(OnProcessExit(target_action=start, on_exit=after_success(TimerAction(period=5., actions=[ekf])))),
        Node(package='tf2_ros', executable='static_transform_publisher',
            arguments=['--frame-id', ns+'/map', '--child-frame-id', ns+'/odom'],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(PythonExpression([str(not zny)]))),
        Node(package='south_lake_sim', executable='fused_odometry_tf.py', namespace=ns,
            parameters=[{'use_sim_time': True}], condition=IfCondition(PythonExpression([str(not zny)]))),
        Node(package='simulation_bringup', executable='visualization_tf.py', namespace=ns,
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        Node(package='rviz2', executable='rviz2', name='simulation_rviz',
            arguments=['-d', namespace_rviz_config(os.path.join(share, 'config', 'go2_sim.rviz'), ns)],
            parameters=[{'use_sim_time': False}],
            remappings=[('/tf', '/'+ns+'/visualization/tf'),
                        ('/tf_static', '/'+ns+'/visualization/tf_static')],
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('south_lake_sim')
    config = lambda name: os.path.join(share, 'config', name)
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='robotdog_nav', choices=['robotdog_nav', 'current']),
        DeclareLaunchArgument('robot_namespace', default_value='', description='Override namespace from params_file launch_config; fallback robot_1'),
        DeclareLaunchArgument('locomotion_mode', default_value=profile_default('hybrid', 'wheels'), choices=['hybrid', 'wheels']),
        DeclareLaunchArgument('world', default_value=profile_default('agriculture.world', 'depot.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('interface', default_value='zny', choices=['zny', 'classic'],
            description='ZNY: wall-stamped sensor contract and navigation-owned TF; classic: legacy sensor format only; navigation is always external.'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory('simulation_bringup'), 'config', 'nav.yaml')),
        DeclareLaunchArgument('spawn_config', default_value=os.path.join(get_package_share_directory('simulation_bringup'), 'config', 'zny_spawn.yaml')),
        DeclareLaunchArgument('sensor_config', default_value=os.path.join(get_package_share_directory('simulation_bringup'), 'config', 'zny_sensors.yaml')),
        *[DeclareLaunchArgument('spawn_'+axis, default_value='') for axis in ('x', 'y', 'z', 'yaw')],
        DeclareLaunchArgument('gait_max_speed', default_value=''),
        DeclareLaunchArgument('gait_max_yaw_rate', default_value=''),
        DeclareLaunchArgument('gps_update_rate', default_value=profile_default('30', '10')),
        DeclareLaunchArgument('gps_position_stddev', default_value=profile_default('0.0002', '0.15')),
        DeclareLaunchArgument('gps_velocity_stddev', default_value='0.03'),
        OpaqueFunction(function=bringup),
    ])
