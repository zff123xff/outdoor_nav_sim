#!/usr/bin/env python3
"""Exercise installed task adapters in an empty ROS domain, without any robot."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from msg_set_msgs.msg import MultiGoal
from execute_task_server.srv import ExecuteTask


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--domain-id',type=int,required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']=str(args.domain_id)
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    discovery=subprocess.run(['ros2','topic','list','--no-daemon','--spin-time','3'],capture_output=True,text=True,timeout=12)
    if discovery.returncode or any(t not in ('/parameter_events','/rosout') for t in discovery.stdout.split()):raise RuntimeError('Audit domain is not empty')
    env=os.environ.copy();env['ROS_LOG_DIR']=str(output/'ros')
    log=(output/'nodes.log').open('w')
    commands=[['ros2','launch','robot_launch','task_pipeline.launch.py','robot_namespace:=robot_1','mqtt:=false'],
              ['ros2','run','multi_goal_to_gp_goal','multi_goal_to_gp_goal','--ros-args','-r','__ns:=/zero_origin','-p','input_topic:=waypoint_goals','-p','output_topic:=gp_goal','-p','reference_latitude:=0.0','-p','reference_longitude:=0.0','-p','reference_altitude:=0.0']]
    processes=[subprocess.Popen(c,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) for c in commands]
    rclpy.init();node=Node('zny_task_audit');goals=[];multi=[];zero=[]
    subs=[node.create_subscription(NavSatFix,'/robot_1/gp_goal',lambda m:goals.append((m.latitude,m.longitude,m.altitude)),10),node.create_subscription(MultiGoal,'/robot_1/waypoint_goals',multi.append,10),node.create_subscription(NavSatFix,'/zero_origin/gp_goal',zero.append,10)]
    client=node.create_client(ExecuteTask,'/agent/robot_1/execute_task')
    odom=node.create_publisher(Odometry,'/robot_1/odom',10)
    zero_pub=node.create_publisher(MultiGoal,'/zero_origin/waypoint_goals',10)
    stop=node.create_publisher(Bool,'/robot_1/target_detected',10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def request(params,mission):
        req=ExecuteTask.Request();req.request_json=json.dumps(dict(mission_id=mission,agent_id='robot_1',capability_id='goto_waypoints',params=params))
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=8)
        if not future.done():raise RuntimeError('ExecuteTask response timeout')
        spin(.6);return json.loads(future.result().response_json)
    result={}
    try:
        if not client.wait_for_service(timeout_sec=20):raise RuntimeError('Task service not available')
        spin(2)
        result['local_service_response']=request(dict(waypoint_frame='local',waypoints=[dict(x=3.,y=0.,z=0.,yaw=1.57,vel=.5)]),'document_local')
        result['local_multigoal_received']=len(multi)
        result['local_gp_goals_published']=len(goals)
        if multi:result['local_flags']={'is_gps_aid':multi[-1].is_gps_aid,'yaw':multi[-1].multi_goal_points[0].yaw,'vel':multi[-1].multi_goal_points[0].vel}
        # Read the actual adapter reference; it is independent from sim_ws' default profile.
        refs={}
        for key in ('reference_latitude','reference_longitude','reference_altitude'):
            p=subprocess.run(['ros2','param','get','/robot_1/multi_goal_to_gp_goal',key],env=env,capture_output=True,text=True,timeout=8)
            refs[key]=float(p.stdout.strip().split()[-1])
        result['pipeline_origin']=refs
        lat,lon,alt=[refs[k] for k in ('reference_latitude','reference_longitude','reference_altitude')]
        result['gps_service_response']=request(dict(waypoint_frame='gps',waypoints=[dict(lat=lat,lon=lon,alt=alt,yaw=1.57,vel=.5),dict(lat=lat,lon=lon+.00001,alt=alt,yaw=-1.57,vel=.2)]),'gps_queue')
        result['gps_goals_before_arrival']=list(goals)
        stop.publish(Bool(data=True));spin(.3)
        # Synthetic odom belongs solely to this message-level fixture: no simulator or nav is launched.
        until=time.monotonic()+1.2
        while time.monotonic()<until:
            m=Odometry();m.header.frame_id='robot_1/odom';m.child_frame_id='robot_1/base_link';m.pose.pose.orientation.w=1.;odom.publish(m);spin(.05)
        result['gps_goals_after_stop_and_arrival']=list(goals)
        result['pending_queue_continues_after_stop']=len(goals)>len(result['gps_goals_before_arrival'])
        # Zero is a valid geodetic origin in the current simulation but the adapter treats it as unset.
        msg=MultiGoal();msg.is_gps_aid=True
        from msg_set_msgs.msg import MultiGoalPoint
        point=MultiGoalPoint();point.x_or_lat=0.;point.y_or_lon=.00001;msg.multi_goal_points=[point]
        zero_pub.publish(msg);spin(1.5)
        result['zero_origin_goals_published']=len(zero)
        result['root_waypoint_subscribers']=node.count_subscribers('/waypoint_goals')
        result['namespaced_waypoint_subscribers']=node.count_subscribers('/robot_1/waypoint_goals')
        result['installed_tests_completed']=True
    finally:
        (output/'result.json').write_text(json.dumps(result,indent=2))
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        node.destroy_node();rclpy.shutdown();log.close()
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
