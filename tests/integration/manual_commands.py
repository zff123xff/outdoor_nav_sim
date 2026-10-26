#!/usr/bin/env python3
"""Exercise real navigation mux/detection wiring in an empty domain; no robot or odometry."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'tests/results/manual_commands.json')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    graph = subprocess.run(['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '3'],
                           capture_output=True, text=True, check=True, timeout=12)
    if any(t not in ('/rosout', '/parameter_events') for t in graph.stdout.split()):
        raise RuntimeError('Test requires an empty isolated domain')
    logs = Path('/tmp/manual_commands_test');logs.mkdir(exist_ok=True)
    children=[];streams=[]
    def start(name, command):
        stream=(logs/(name+'.log')).open('w');streams.append(stream)
        p=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p);return p
    def stop(p):
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    nav=start('navigation',['bash',str(ROOT.parent/'outdoor_nav/launch_hil.sh'),
                            'robot_namespace:=command_test','use_foxglove:=false'])
    rclpy.init();node=rclpy.create_node('manual_commands_fixture')
    received={'out':[],'remote':[],'detect':[],'stop':[]}
    subscriptions=[]
    for key,typ,topic in [('out',Twist,'cmd_vel'),('remote',Twist,'cmd_vel_remote'),
                          ('detect',Bool,'target_detected'),('stop',Bool,'stop_planner')]:
        subscriptions.append(node.create_subscription(typ,'/command_test/'+topic,
                             lambda m,k=key:received[k].append((time.monotonic(),m)),10))
    background=node.create_publisher(Twist,'/command_test/cmd_vel_nav',10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            background.publish(Twist(linear=Vector3(x=.1)))
            rclpy.spin_once(node,timeout_sec=.02)
    def wait(check,label,seconds=15):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            spin(.025)
            if check():return
        raise TimeoutError(label+'; inspect '+str(logs))
    def send(name,mode,*extra):
        return start(name,['bash',str(ROOT/'commands/send.sh'),mode,'--namespace','command_test',*extra])
    def value(items,x):return any(abs(m.linear.x-x)<1e-6 for _,m in items)
    result={'domain_id':args.domain_id,'no_robot_or_odometry':True}
    try:
        wait(lambda:node.count_subscribers('/command_test/cmd_vel_remote')>0 and
             node.count_subscribers('/command_test/target_detected')>=1,'remote input and original detection consumer')
        wait(lambda:value(received['out'],.1),'background navigation velocity')
        assert node.count_subscribers('/command_test/yolo_detect') == 0, 'navigation still exposes YOLO alias'
        assert not any(e.node_name == 'twist_mux' for e in node.get_subscriptions_info_by_topic('/command_test/target_detected')), 'mux still has detection lock'
        result['navigation_restored']='no yolo_detect subscription or target_detected mux lock'
        start_index=len(received['remote'])
        velocity=send('velocity','velocity','--vx','.3','--duration','1')
        wait(lambda:value(received['out'],.3),'manual priority over navigation')
        wait(lambda:velocity.poll() is not None,'bounded velocity exit')
        assert velocity.returncode==0
        assert received['remote'][-1][1].linear.x==0.
        assert len(received['remote'])-start_index>=10
        result['bounded_velocity']='manual 0.3 m/s preempted navigation 0.1 m/s and ended with zero'
        active=send('active_velocity','velocity','--vx','.2','--duration','10')
        wait(lambda:value(received['out'],.2),'active manual velocity')
        stop_start=len(received['stop'])
        detection=send('yolo_stop','yolo_detect')
        wait(lambda:any(m.data for _,m in received['detect']),'YOLO detection event')
        wait(lambda:detection.poll() is not None and active.poll() is not None,'YOLO stopped manual sender')
        assert detection.returncode==0 and active.returncode==0
        wait(lambda:'[yolo] Target detected!' in (logs/'navigation.log').read_text(), 'real detection callback')
        assert any(m.data for _,m in received['stop'][stop_start:]),'no planner cancellation'
        result['target_detected_stop']='original input bridge emitted stop_planner; active manual sender exited; no mux lock added'
        clear=send('detection_false','yolo_detect','--clear')
        wait(lambda:clear.poll() is not None,'detection false sender exit')
        assert clear.returncode==0
        assert any(not m.data for _,m in received['detect'])
        result['target_detected_false']='false sent; no navigation resume or lock semantics'
        interrupted=send('interrupted_velocity','velocity','--vx','.25','--duration','10')
        wait(lambda:value(received['out'],.25),'velocity before SIGINT')
        os.killpg(interrupted.pid,signal.SIGINT)
        wait(lambda:interrupted.poll() is not None,'SIGINT cleanup')
        assert interrupted.returncode==0 and received['remote'][-1][1].linear.x==0.
        result['sigint']='sender published zero and exited normally'
        result['passed']=True
    finally:
        for p in reversed(children):stop(p)
        for stream in streams:stream.close()
        node.destroy_node();rclpy.try_shutdown()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
