#!/usr/bin/env python3
"""Effective Classic model, exclusive controller ownership and origin checks."""
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

spec=importlib.util.spec_from_file_location('sim_launch',sys.argv[1])
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
for profile in ('robotdog_nav','current'):
    context=LaunchContext();context.launch_configurations.update(profile=profile,gui='false',interface='classic',robot_namespace='robot',spawn_config=module.get_package_share_directory('south_lake_sim')+'/config/'+('robotdog_spawn.yaml' if profile=='robotdog_nav' else 'sim_spawn.yaml'),sensor_config=module.get_package_share_directory('south_lake_sim')+'/config/robotdog_sensors.yaml')
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    nodes=[];original=module.Node
    def capture(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)
    with patch.object(module,'Node',side_effect=capture):module.bringup(context)
    rsp=next(n for n in nodes if n.get('package')=='robot_state_publisher')
    model=ET.fromstring(rsp['parameters'][0]['robot_description'])
    plugins=[p.get('filename') for p in model.findall('.//plugin') if p.get('filename')]
    assert 'libgazebo_ros2_control.so' in plugins and 'libmid360_plugin.so' in plugins
    assert 'libpx4_classic_sensors.so' in plugins
    assert not any('ignition-gazebo' in p or 'gz_ros2_control' in p or 'unilidar' in p for p in plugins)
    assert not any(n.get('package')=='ros_gz_bridge' for n in nodes)
    gps=model.find(".//plugin[@name='px4_sim_sensors']")
    imu=model.find(".//plugin[@name='native_imu_ros']/frame_name")
    assert imu.text=='robot/imu_link'
    imu_sensor=model.find(".//sensor[@name='native_imu']")
    for quantity in ('angular_velocity','linear_acceleration'):
        for axis in ('x','y','z'):
            noise=imu_sensor.find('imu/'+quantity+'/'+axis+'/noise')
            assert noise is not None and noise.get('type')=='gaussian'
            assert float(noise.findtext('mean'))==0 and float(noise.findtext('stddev'))==0
    assert model.find(".//plugin[@name='mid360_ros']/frame_name").text=='robot/livox_frame'
    controllers=[n['arguments'][0] for n in nodes if n.get('executable')=='spawner']
    executables=[n.get('executable') for n in nodes]
    leg_interface=model.find("ros2_control/joint[@name='FL_hip_joint']/command_interface").get('name')
    if profile=='robotdog_nav':
        assert controllers==['joint_state_broadcaster','leg_trajectory_controller','wheel_velocity_controller']
        assert 'hybrid_motion_controller.py' in executables and 'stand_initializer.py' not in executables
        assert leg_interface=='effort'
        assert float(gps.findtext('reference_latitude'))==40.9445353
        assert float(gps.findtext('gps_update_rate'))==30
        assert context.launch_configurations['world']=='agriculture.world'
        assert float(model.find(".//plugin[@name='mid360_ros']/ray/range/max").text)==5
    else:
        assert controllers==['joint_state_broadcaster','leg_position_controller','diff_drive_controller']
        assert 'stand_initializer.py' in executables and 'hybrid_motion_controller.py' not in executables
        assert leg_interface=='position'
        assert context.launch_configurations['world']=='depot.world'
    print(profile,'Classic model / control mode / sensors passed')

rviz_source=Path(module.get_package_share_directory('south_lake_sim'))/'config/go2_sim.rviz'
for namespace in ('robot','robot_a','robot_b'):
    path=Path(module.namespace_rviz_config(rviz_source,namespace))
    config=yaml.safe_load(path.read_text())
    manager=config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame']==namespace+'/map'
    robot=next(d for d in manager['Displays'] if d['Class']=='rviz_default_plugins/RobotModel')
    assert robot['TF Prefix']==namespace
    assert robot['Description Topic']['Value']=='/robot_description'
    goal=next(t for t in manager['Tools'] if t['Class']=='rviz_default_plugins/SetGoal')
    assert goal['Topic']=='/'+namespace+'/goal_pose'
    if namespace!='robot':
        assert '/robot/' not in path.read_text() and 'robot/map' not in path.read_text()
    path.unlink()
    print(namespace,'RViz frames / robot prefix / display and goal topics passed')
