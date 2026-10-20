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
    context=LaunchContext();context.launch_configurations.update(profile=profile,gui='false')
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
    assert model.find(".//plugin[@name='mid360_ros']/frame_name").text=='robot/livox_frame'
    controllers=[n['arguments'][0] for n in nodes if n.get('executable')=='spawner']
    executables=[n.get('executable') for n in nodes]
    leg_interface=model.find("ros2_control/joint[@name='FL_hip_joint']/command_interface").get('name')
    if profile=='robotdog_nav':
        assert controllers==['joint_state_broadcaster','leg_trajectory_controller','wheel_velocity_controller']
        assert 'hybrid_motion_controller.py' in executables and 'stand_initializer.py' not in executables
        assert leg_interface=='effort'
        assert float(gps.findtext('reference_latitude'))==0
        assert float(gps.findtext('gps_update_rate'))==30
        assert context.launch_configurations['world']=='agriculture.world'
        assert float(model.find(".//plugin[@name='mid360_ros']/ray/range/max").text)==5
    else:
        assert controllers==['joint_state_broadcaster','leg_position_controller','diff_drive_controller']
        assert 'stand_initializer.py' in executables and 'hybrid_motion_controller.py' not in executables
        assert leg_interface=='position'
        assert context.launch_configurations['world']=='depot.world'
    print(profile,'Classic model / control mode / sensors passed')

nav_spec=importlib.util.spec_from_file_location('nav_launch',Path(sys.argv[1]).with_name('go2_px4_nav.launch.py'))
nav_module=importlib.util.module_from_spec(nav_spec);nav_spec.loader.exec_module(nav_module)
for profile in ('robotdog_nav','current'):
    context=LaunchContext();context.launch_configurations['profile']=profile
    for action in nav_module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    with patch.object(nav_module,'Node') as create:nav_module.bringup(context)
    bridge=next(c.kwargs for c in create.call_args_list if c.kwargs.get('name')=='nav2_input_bridge')
    params=yaml.safe_load(Path(bridge['parameters'][0]).read_text())
    values=params['/robot/nav2_input_bridge']['ros__parameters']
    assert values['reference_latitude']==(0. if profile=='robotdog_nav' else 40.9445353)
    assert values['use_sim_time'] and values['local_odom_topic']=='mavros/odom'
    Path(bridge['parameters'][0]).unlink()
    print(profile,'navigation reference passed')
