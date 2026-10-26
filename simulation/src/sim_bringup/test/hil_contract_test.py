#!/usr/bin/env python3
"""Check exclusive command/odom ownership and the NX HIL dispatch boundary."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

source = Path(sys.argv[1]).resolve()
spec = importlib.util.spec_from_file_location('hil_sim', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
nodes = []
original = module.Node

def capture(**kwargs):
    nodes.append(kwargs)
    return original(**kwargs)

# ZNY is a separate boundary: wall-stamped sensor outputs, private model TF,
# fixed documented mounting, and no environment-side navigation arbiter.
context = LaunchContext()
context.launch_configurations.update(interface='zny',
    navigation='false', gui='false', robot_namespace='robot', spawn_config=str(source.parents[4]/'simulation/config/zny_spawn.yaml'))
for action in module.generate_launch_description().entities:
    if isinstance(action, DeclareLaunchArgument):
        action.execute(context)
nodes.clear()
with patch.object(module, 'Node', side_effect=capture):
    module.bringup(context)
rsp = next(node for node in nodes if node.get('package') == 'robot_state_publisher')
assert rsp['remappings'] == [('/tf', '/robot/simulation/tf'), ('/tf_static', '/robot/simulation/tf_static')]
model = ET.fromstring(rsp['parameters'][0]['robot_description'])
mount = model.find("joint[@name='lidar_joint']/origin")
xyz = list(map(float, mount.get('xyz').split()))
body_z = float(model.find("joint[@name='go2w_body_joint']/origin").get('xyz').split()[2])
assert abs(xyz[0]-.18) < 1e-9 and abs(xyz[2]+body_z-.53) < 1e-9
assert abs(float(mount.get('rpy').split()[1])-.3141592653589793) < 1e-9
assert model.find(".//plugin[@name='mid360_ros']/zny_interface").text == 'true'
ekf = next(node for node in nodes if node.get('executable') == 'px4_ekf_node')
assert ekf['parameters'][0]['zny_interface'] is True
assert ekf['parameters'][0]['odom_publish_rate'] == 100.
tf_nodes = [n for n in nodes if n.get('executable') in ('fused_odometry_tf.py', 'static_transform_publisher')]
assert all(not n['condition'].evaluate(context) for n in tf_nodes), 'ZNY simulation must not publish localization TF'
drive = next(node for node in nodes if node.get('executable') == 'hybrid_motion_controller.py')
assert drive['parameters'][1]['max_linear_speed'] == 1.5
assert drive['parameters'][1]['max_lateral_speed'] == .6
assert drive['parameters'][1]['max_angular_speed'] == 1.
print('ZNY environment boundary, mounting and limits: passed')
context.launch_configurations['locomotion_mode'] = 'wheels'
try:
    module.bringup(context)
except RuntimeError as error:
    assert 'lateral velocity' in str(error)
else:
    raise AssertionError('ZNY accepted a drive mode without lateral control')


rviz = next(n for n in nodes if n.get('package') == 'rviz2')
assert rviz['condition'].evaluate(context)
assert rviz['parameters'] == [{'use_sim_time': False}]
assert rviz['remappings'][0] == ('/tf', '/robot/visualization/tf')
context.launch_configurations['rviz'] = 'false'
assert not rviz['condition'].evaluate(context)
print('RViz default startup, private TF and headless switch: passed')

assert not any(n.get('package') == 'twist_mux' for n in nodes), 'simulation must not start a competing mux'
assert ekf['remappings'] == [('odom', 'simulation/ekf_odom')]
assert all(a.name != 'external_navigation' for a in module.generate_launch_description().entities if isinstance(a, DeclareLaunchArgument))
