#!/usr/bin/env python3
"""Exercise the real Nav2 manager against fault-injected lifecycle servers."""
import importlib.util
import os
from pathlib import Path
import signal
import sys
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry

spec = importlib.util.spec_from_file_location('relay', Path(__file__).resolve().parents[1]/'scripts/lifecycle_service_relay.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
tf_spec = importlib.util.spec_from_file_location('fused_tf', Path(__file__).resolve().parents[2]/'sim_world/scripts/fused_odometry_tf.py')
tf_module = importlib.util.module_from_spec(tf_spec)
tf_spec.loader.exec_module(tf_module)


class FusedTFRegression(unittest.TestCase):
    def test_below_datum_pose_is_not_flattened(self):
        odom = Odometry()
        odom.header.frame_id = 'robot/odom'
        odom.header.stamp.sec = 123
        odom.child_frame_id = 'robot/base_link'
        odom.pose.pose.position.z = -.64
        odom.pose.pose.orientation.w = 1.0
        transform = tf_module.odometry_transform(odom)
        self.assertEqual(transform.transform.translation.z, -.64)
        self.assertEqual(transform.header, odom.header)
        self.assertEqual(transform.child_frame_id, odom.child_frame_id)

    def test_invalid_height_is_rejected(self):
        odom = Odometry()
        odom.pose.pose.position.z = float('nan')
        with self.assertRaises(ValueError): tf_module.odometry_transform(odom)


class FaultServer(Node):
    def __init__(self, mode):
        super().__init__('fault_server')
        self.mode = mode
        self.states = {'controller_server': 1, 'straight_planner': 1}
        self.counts = {}
        self.reads = 0
        self.group = ReentrantCallbackGroup()
        self.servers = []
        for name in self.states:
            self.servers.extend([
                self.create_service(ChangeState, name+'/change_state',
                    lambda req,res,n=name:self.change(n,req,res), callback_group=self.group),
                self.create_service(GetState, name+'/get_state',
                    lambda req,res,n=name:self.get(n,req,res), callback_group=self.group)])

    def change(self, name, request, response):
        key = (name, request.transition.id)
        self.counts[key] = self.counts.get(key, 0)+1
        if self.mode == 'reject' or self.mode == 'no_execution':
            if self.mode == 'no_execution': time.sleep(.6)
            response.success = False
            return response
        target = module.TARGETS[request.transition.id]
        if self.states[name] == target:
            response.success = False  # Replaying would fail on real lifecycle nodes.
            return response
        self.states[name] = target
        if self.mode == 'lost_transition': time.sleep(.6)
        response.success = True
        return response

    def get(self, name, request, response):
        self.reads += 1
        if self.mode == 'lost_read' and self.reads == 1: time.sleep(.5)
        response.current_state.id = self.states[name]
        return response


class RelayIntegration(unittest.TestCase):
    def run_case(self, mode, expected):
        ns = '/relay_test_'+mode
        rclpy.init(args=['--ros-args','-r','__ns:='+ns,
                         '-p','response_timeout:=0.3','-p','query_timeout:=0.15','-p','query_attempts:=3'])
        relay = module.LifecycleRelay()
        server = FaultServer(mode)
        observer = Node('observer')
        executor = MultiThreadedExecutor(num_threads=12)
        for node in (relay,server,observer): executor.add_node(node)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        log = tempfile.TemporaryFile(mode='w+')
        command=['ros2','run','nav2_lifecycle_manager','lifecycle_manager','--ros-args',
                 '-r','__ns:='+ns,'-r','__node:=manager',
                 '-p','autostart:=true','-p','bond_timeout:=0.0',
                 '-p','node_names:=[controller_server, straight_planner]']
        for name in server.states:
            for service in ('change_state','get_state'):
                command += ['-r',name+'/'+service+':=lifecycle_relay/'+name+'/'+service]
        process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            active = observer.create_client(Trigger,'manager/is_active')
            deadline = time.monotonic()+10
            success = False
            text = ''
            while time.monotonic()<deadline:
                time.sleep(.1)
                log.seek(0); text=log.read()
                if 'Failed to bring up all requested nodes' in text: break
                if active.service_is_ready():
                    result=relay.call(active,Trigger.Request(),.4)
                    if result and result.success:
                        success=True
                        break
            self.assertEqual(success,expected,text)
            if expected:
                self.assertEqual(list(server.states.values()),[3,3])
                self.assertEqual(server.counts,{(n,t):1 for n in server.states for t in (1,3)})
            else:
                self.assertIn('Failed to bring up all requested nodes',text)
                self.assertEqual(server.states['controller_server'],1)
                self.assertEqual(server.counts,{('controller_server',1):1})
            watchdog = subprocess.run([
                sys.executable, str(Path(__file__).resolve().parents[1]/'scripts/lifecycle_startup_watchdog.py'),
                '--ros-args','-r','__ns:='+ns,
                '-r','lifecycle_manager_navigation/is_active:=manager/is_active',
                '-p','startup_timeout:=1.0'], capture_output=True, text=True, timeout=4)
            self.assertEqual(watchdog.returncode,0 if expected else 1,watchdog.stderr)
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
            log.close()
            executor.shutdown();thread.join()
            for node in (relay,server,observer):node.destroy_node()
            rclpy.shutdown()

    def test_lost_transition_replies(self): self.run_case('lost_transition',True)
    def test_lost_read_reply(self): self.run_case('lost_read',True)
    def test_explicit_rejection(self): self.run_case('reject',False)
    def test_timeout_without_execution(self): self.run_case('no_execution',False)

    def test_watchdog_bounds_missing_manager(self):
        result = subprocess.run([
            sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/lifecycle_startup_watchdog.py'),
            '--ros-args','-r','__ns:=/relay_test_absent_manager',
            '-p','startup_timeout:=0.5'],capture_output=True,text=True,timeout=3)
        self.assertEqual(result.returncode,1,result.stderr)
        self.assertIn('deadline exceeded',result.stderr)


if __name__ == '__main__': unittest.main()
