#!/usr/bin/env python3
"""Check command interfaces in an empty domain, without robot or fake odometry."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from msg_set_msgs.msg import MultiGoal, MultiTask
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'tests/results/command_interfaces.json')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    graph = subprocess.run(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '3'],
                           capture_output=True, text=True, timeout=12, check=True)
    if any(topic not in ('/parameter_events', '/rosout') for topic in graph.stdout.split()):
        raise RuntimeError('Test requires an empty ROS domain')
    # Never send test requests through an already running broker.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 18884))
    logs = Path('/tmp/outdoor_nav_command_interfaces')
    logs.mkdir(exist_ok=True)
    # Keep user-edited task examples unchanged; generate deterministic test inputs separately.
    inputs = logs/'generated_inputs'
    subprocess.run(['python3', str(ROOT/'commands/scripts/generate_examples.py'),
                    '--data-dir', str(inputs)], capture_output=True, text=True, check=True, timeout=10)
    processes, streams = [], []
    os.environ['ROS_LOG_DIR'] = str(logs/'ros')
    rclpy.init()
    node = rclpy.create_node('command_interface_test')
    clock = node.create_publisher(Clock, '/clock', 10)
    received = {name: [] for name in ['gps', 'uav', 'pose', 'waypoints', 'multitask', 'fcu']}
    subscriptions = []
    for topic, kind, key in [('gp_goal', NavSatFix, 'gps'), ('uav/target_gps', NavSatFix, 'uav'),
                             ('goal_pose', PoseStamped, 'pose'), ('waypoint_goals', MultiGoal, 'waypoints'),
                             ('swarms_backend/global/multitask', MultiTask, 'multitask'), ('mavros/state', State, 'fcu')]:
        subscriptions.append(node.create_subscription(kind, '/robot_1/'+topic,
                             lambda message, k=key: received[k].append(message), 10))
    def start(name, command):
        stream = (logs/(name+'.log')).open('w')
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    def wait(predicate, label, timeout=25):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            message = Clock()
            ns = time.monotonic_ns()
            message.clock.sec, message.clock.nanosec = divmod(ns, 10**9)
            clock.publish(message)
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise TimeoutError(label+'; inspect '+str(logs))
    def finish(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    results = {'domain_id': args.domain_id, 'no_robot_or_odometry': True}
    try:
        reference = yaml.safe_load((ROOT/'simulation/config/nav.yaml').read_text())['nav2_input_bridge']['ros__parameters']
        command = ['ros2', 'run', 'nav2_input_bridge', 'input_bridge_node', '--ros-args',
                   '-r', '__ns:=/robot_1', '-p', 'use_sim_time:=true', '-p', 'require_nav_sat_fix:=false']
        for key in ['reference_latitude', 'reference_longitude', 'reference_altitude']:
            command.extend(['-p', f'{key}:={reference[key]}'])
        start('input_bridge', command)
        stack = start('task_receivers', ['bash', str(ROOT/'commands/start.sh'), 'tasks'])
        wait(lambda: node.count_subscribers('/robot_1/gp_goal') > 0 and
             node.count_subscribers('/robot_1/waypoint_goals') > 0, 'task endpoints')
        # Both public single-GPS inputs must reach the original GPS->ENU bridge.
        for route, key in [('gp_goal', 'gps'), ('uav/target_gps', 'uav')]:
            old_pose = len(received['pose'])
            process = start(key, ['bash', str(ROOT/'commands/send.sh'), route, '--file', str(inputs/'gps_goal.yaml')])
            wait(lambda: process.poll() is not None, route+' sender')
            assert process.returncode == 0, route
            wait(lambda: len(received['pose']) > old_pose and bool(received[key]), route+' conversion')
            pose = received['pose'][-1].pose.position
            assert abs(pose.x-1) < .1 and abs(pose.y) < .1, (route, pose)
            results[route] = 'received and converted to ENU'
        for route in ['waypoints', 'task']:
            old_count = len(received['waypoints'])
            process = start(route, ['bash', str(ROOT/'commands/send.sh'), route,
                                    '--file', str(inputs/('multi_goal.yaml' if route == 'waypoints' else 'execute_task.yaml'))])
            wait(lambda: len(received['waypoints']) > old_count, route+' MultiGoal')
            assert process.poll() is None, route+' failed before keepalive'
            assert len(received['waypoints'][-1].multi_goal_points) == 2
            results[route] = 'two GPS waypoints received by original queue'
            finish(process)
        # Keep the negative examples executable and verify their actual rejection layer.
        old_count = len(received['waypoints'])
        wrong_agent = start('negative_wrong_agent', ['bash', str(ROOT/'commands/send.sh'), 'task',
                            '--file', str(inputs/'negative/wrong_agent.yaml'), '--dry-run'])
        wait(lambda: wrong_agent.poll() is not None, 'wrong-agent preflight rejection')
        assert wrong_agent.returncode != 0
        assert 'agent_id must match' in (logs/'negative_wrong_agent.log').read_text()
        assert len(received['waypoints']) == old_count
        results['negative_wrong_agent'] = 'rejected before ROS submission'
        unsupported = start('negative_unsupported_task', ['bash', str(ROOT/'commands/send.sh'), 'task',
                            '--file', str(inputs/'negative/unsupported_task.yaml')])
        wait(lambda: unsupported.poll() is not None, 'unsupported capability service rejection')
        assert unsupported.returncode != 0
        assert 'Unsupported capability' in (logs/'negative_unsupported_task.log').read_text()
        assert len(received['waypoints']) == old_count
        results['negative_unsupported_task'] = 'original ExecuteTask service rejected capability'
        old_gps = len(received['gps'])
        local = start('negative_local_waypoints', ['bash', str(ROOT/'commands/send.sh'), 'waypoints',
                      '--file', str(inputs/'negative/local_multi_negative.yaml')])
        wait(lambda: len(received['waypoints']) > old_count and
             'is_gps_aid=False (local frame)' in (logs/'task_receivers.log').read_text(),
             'original GPS queue rejection of local waypoints')
        assert received['waypoints'][-1].is_gps_aid is False
        assert len(received['gps']) == old_gps
        assert local.poll() is None
        results['negative_local_waypoints'] = 'original GPS queue skipped local waypoints; no GPS goal emitted'
        finish(local)
        broker = shutil.which('mosquitto') or str(ROOT/'deps/mqtt/usr/sbin/mosquitto')
        start('broker', [broker, '-c', str(ROOT/'commands/config/mqtt/broker_test.conf')])
        start('mqtt_receivers', ['bash', str(ROOT/'commands/start.sh'), 'mqtt-gateway',
              'mqtt_config:='+str(ROOT/'commands/config/mqtt/test_client.yaml'), 'start_pipeline:=false'])
        wait(lambda: node.count_subscribers('/robot_1/swarms_backend/global/multitask') >= 2, 'MQTT task parser')
        old_count = len(received['waypoints'])
        process = start('mqtt_send', ['bash', str(ROOT/'commands/send.sh'), 'mqtt',
                        '--config', str(ROOT/'commands/config/mqtt/test_client.yaml'),
                        '--file', str(inputs/'mqtt_task_stream.json')])
        wait(lambda: process.poll() is not None, 'MQTT application ACK')
        assert process.returncode == 0, 'MQTT sender failed'
        wait(lambda: bool(received['multitask']) and len(received['waypoints']) > old_count, 'MQTT to MultiGoal')
        assert not received['fcu'], 'simulated FCU status was published'
        assert stack.poll() is None
        results['mqtt'] = 'protocol ACK and MultiTask -> ExecuteTask -> MultiGoal received'
        results['mock_fcu_messages'] = 0
        results['passed'] = True
    finally:
        for process in reversed(processes):
            finish(process)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
