#!/usr/bin/env python3
"""Start missing task services/queue and submit one ExecuteTask YAML request."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rcl_interfaces.srv import GetParameters
import yaml
from execute_task_server.srv import ExecuteTask

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['execute-task', 'multi-goal'], default='execute-task')
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--file', type=Path)
    parser.add_argument('--params-file', type=Path, default=ROOT.parent/'simulation/config/nav.yaml')
    parser.add_argument('--serve-only', action='store_true', help='Start/reuse task service and queue without sending a task')
    parser.add_argument('--dry-run', action='store_true', help='Validate input and print it without ROS communication')
    parser.add_argument('--use-sim-time', action='store_true', help='Legacy Classic mode only: require /clock')
    args = parser.parse_args()
    if args.file is None:
        args.file = ROOT/'examples'/('multi_goal.yaml' if args.mode == 'multi-goal' else 'execute_task.yaml')
    ns = args.namespace.strip('/')
    if not ns or '/' in ns:
        parser.error('namespace must be a single nonempty ROS name')
    contents = yaml.safe_load(args.file.read_text()) if not args.serve_only else None
    if args.mode == 'execute-task' and not args.serve_only:
        request_json = contents['request_json']
        payload = json.loads(request_json)
        if payload.get('agent_id') != ns:
            parser.error('YAML request_json.agent_id must match --namespace')
    elif not args.serve_only:
        from msg_set_msgs.msg import MultiGoal
        from rosidl_runtime_py.set_message import set_message_fields
        message = MultiGoal()
        set_message_fields(message, copy.deepcopy(contents))
        if not message.multi_goal_points:
            parser.error('YAML contains no waypoints')
    reference = yaml.safe_load(args.params_file.read_text())['nav2_input_bridge']['ros__parameters']
    if args.dry_run:
        print(json.dumps({'namespace': ns, 'mode': args.mode, 'input': contents,
                          'serve_only': args.serve_only}, ensure_ascii=False, indent=2))
        return
    children = []
    rclpy.init()
    node = rclpy.create_node('execute_task_yaml_runner')
    client = node.create_client(ExecuteTask, f'/agent/{ns}/execute_task')

    def launch_node(package, executable, **parameters):
        command = ['ros2', 'run', package, executable, '--ros-args',
                   '-r', '__ns:=/'+ns, '-p', 'use_sim_time:='+str(args.use_sim_time).lower()]
        for key, value in parameters.items():
            command.extend(['-p', f'{key}:={value}'])
        children.append(subprocess.Popen(command, start_new_session=True))

    def wait_for(check, label, seconds=30):
        deadline = time.monotonic()+seconds
        while not check():
            if any(p.poll() is not None for p in children):
                raise RuntimeError('A task launch exited before submission')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Timed out waiting for {label}')
            rclpy.spin_once(node, timeout_sec=0.1)

    try:
        # Give existing processes time to be discovered; do not create competing queues.
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if args.mode == 'execute-task' and not client.service_is_ready():
            launch_node('execute_task_server', 'execute_task_node', agent_id=ns,
                        default_waypoint_frame='gps')
        elif args.mode == 'execute-task':
            print('Reusing existing ExecuteTask service', flush=True)
        if ('multi_goal_to_gp_goal', '/'+ns) not in node.get_node_names_and_namespaces():
            launch_node('multi_goal_to_gp_goal', 'multi_goal_to_gp_goal',
                        input_topic='waypoint_goals', output_topic='gp_goal', odom_topic='odom',
                        **{key: reference[key] for key in
                           ('reference_latitude', 'reference_longitude', 'reference_altitude')})
        else:
            print('Reusing existing GPS waypoint queue (its current origin/queue are preserved)', flush=True)
        if args.mode == 'execute-task':
            wait_for(client.service_is_ready, 'ExecuteTask service')
        # Wait for the service publisher to match the queue before calling: volatile messages are sent once.
        topic = f'/{ns}/waypoint_goals'
        publisher = node.create_publisher(MultiGoal, topic, 10) if args.mode == 'multi-goal' and not args.serve_only else None
        wait_for(lambda: (args.mode == 'multi-goal' or any(e.node_name == 'execute_task_node' and e.node_namespace == '/'+ns
                             for e in node.get_publishers_info_by_topic(topic)))
                 and any(e.node_name == 'multi_goal_to_gp_goal' and e.node_namespace == '/'+ns
                         for e in node.get_subscriptions_info_by_topic(topic)), 'waypoint endpoints')
        keys = ['reference_latitude', 'reference_longitude', 'reference_altitude']
        for target in ['multi_goal_to_gp_goal', 'nav2_input_bridge']:
            if (target, '/'+ns) not in node.get_node_names_and_namespaces():
                continue
            parameters = node.create_client(GetParameters, '/'+ns+'/'+target+'/get_parameters')
            wait_for(parameters.service_is_ready, target+' parameter service')
            request = GetParameters.Request()
            request.names = keys
            deadline = time.monotonic()+30
            actual = None
            while time.monotonic() < deadline:
                response = parameters.call_async(request)
                try:
                    wait_for(response.done, target+' reference parameters',
                             seconds=min(2, max(0, deadline-time.monotonic())))
                except TimeoutError:
                    # Reads are safe to retry after navigation restarts or a
                    # request/reply is lost. Never replay ExecuteTask this way.
                    parameters.remove_pending_request(response)
                    response.cancel()
                    continue
                actual = response.result().values
                break
            if actual is None:
                raise TimeoutError(
                    f'{target} parameter service was discovered but did not reply '
                    'after repeated queries; no task sent. Check navigation health '
                    'and the RMW/DDS settings on both hosts.')
            if len(actual) != len(keys) or any(value.type != 3 or abs(value.double_value-float(reference[key])) > 1e-7
                   for key, value in zip(keys, actual)):
                raise RuntimeError(target+' GPS origin differs from --params-file; restart with the shared configuration')
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if args.serve_only:
            print('Task service and GPS queue ready; no task sent.', flush=True)
        elif publisher is not None:
            wait_for(lambda: publisher.get_subscription_count() > 0, 'matched waypoint subscriber')
            publisher.publish(message)
            print(f'Published {len(message.multi_goal_points)} waypoints once from {args.file}', flush=True)
        else:
            request = ExecuteTask.Request()
            request.request_json = request_json
            future = client.call_async(request)
            wait_for(future.done, 'ExecuteTask response')
            response = future.result().response_json
            print(response, flush=True)
            if json.loads(response).get('status') != 'success':
                raise RuntimeError('ExecuteTask rejected the request')
        if not args.serve_only:
            print('Task submitted once; success means accepted, not arrived. Ctrl+C stops launches started here.', flush=True)
        while rclpy.ok():
            if any(p.poll() is not None for p in children):
                raise RuntimeError('A task launch exited')
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for process in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
