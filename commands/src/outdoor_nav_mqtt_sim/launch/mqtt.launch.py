"""Hardware MQTT protocol -> hardware task service -> hardware GPS queue."""
import re
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bringup(context):
    def value(key): return LaunchConfiguration(key).perform(context)
    ns = value('robot_namespace').strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        raise RuntimeError('robot_namespace must be one ROS name')
    with open(value('mqtt_config')) as stream: mqtt = yaml.safe_load(stream)
    with open(value('nav_params')) as stream: nav = yaml.safe_load(stream)
    reference = {k: float(nav['nav2_input_bridge']['ros__parameters'][k])
                 for k in ('reference_latitude', 'reference_longitude', 'reference_altitude')}
    if abs(reference['reference_latitude']) < 1e-9 and abs(reference['reference_longitude']) < 1e-9:
        raise RuntimeError('Hardware GPS task queue rejects origin (0,0); use the shared acceptance nav.yaml')
    identifier = int(mqtt.get('vehicle_id', 1))
    if not 0 <= identifier <= 127:
        raise RuntimeError('vehicle_id must fit the hardware MultiTask int8 drones_id')
    sim_time = value('use_sim_time').lower() == 'true'
    pipeline = IfCondition(LaunchConfiguration('start_pipeline'))
    nodes = [
        Node(package='outdoor_nav_mqtt_sim', executable='mqtt_gateway', namespace=ns,
             parameters=[{'prefix': '', 'curr_drone_id': '/'+ns,
                          'vehicle_id': identifier, 'address': str(mqtt['address']),
                          'mqtt_username': str(mqtt.get('mqtt_username', '')),
                          'mqtt_passwd': str(mqtt.get('mqtt_passwd', '')),
                          'mqtt_buffer_size': 100, 'mqtt_timeout': 10,
                          'mqtt_keep_alive_interval': 30, 'use_sim_time': sim_time}], output='screen'),
        Node(package='execute_task_server', executable='execute_task_node', namespace=ns,
             parameters=[{'agent_id': ns, 'default_waypoint_frame': 'gps', 'use_sim_time': sim_time}],
             condition=pipeline, output='screen'),
        Node(package='multi_goal_to_gp_goal', executable='multi_goal_to_gp_goal', namespace=ns,
             parameters=[dict(reference, input_topic='waypoint_goals', output_topic='gp_goal',
                              odom_topic='odom', use_sim_time=sim_time)],
             condition=pipeline, output='screen'),
        Node(package='outdoor_nav_mqtt_sim', executable='task_bridge.py',
             name='multi_task_bridge_'+ns,
             parameters=[{'drone_name': ns, 'drone_id': identifier, 'use_sim_time': sim_time,
                          'flight_alt_m': reference['reference_altitude']}],
             remappings=[('/mavros/global_position/global', '/'+ns+'/sensors/gps/fix')], output='screen'),
    ]
    def failed(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='MQTT pipeline node exited with an error'))]
        return []
    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=failed)) for node in nodes]+nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='robot_1'),
        DeclareLaunchArgument('mqtt_config', description='MQTT address, credentials and vehicle_id YAML'),
        DeclareLaunchArgument('nav_params', description='Same navigation YAML as simulation and navigation'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_pipeline', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=bringup),
    ])
