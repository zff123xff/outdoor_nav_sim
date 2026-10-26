#!/usr/bin/env python3
"""Inspect a running isolated ZNY simulation; --move explicitly tests a 1 m goal."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace',default='robot_1')
    parser.add_argument('--require-nav',action='store_true')
    parser.add_argument('--move',action='store_true')
    parser.add_argument('--output',type=Path,default=ROOT/'tests/results/zny_live_check.json')
    args=parser.parse_args()
    ns=args.namespace.strip('/')
    rclpy.init();node=Node('zny_live_checker')
    data={};arrivals={};statuses={};sent_stamp=0
    def receive(key,message):
        data[key]=message
        arrivals.setdefault(key,[]).append(time.monotonic())
    subscriptions=[]
    for key,typ,topic in [('odom',Odometry,'mavros/odom'),('cloud',PointCloud2,'livox/lidar'),
                          ('status',String,'ekf/status'),('truth',Odometry,'simulation/truth/odom'),('cmd',Twist,'cmd_vel')]:
        subscriptions.append(node.create_subscription(typ,'/'+ns+'/'+topic,lambda m,k=key:receive(k,m),qos_profile_sensor_data))
    def action(message):
        for entry in message.status_list:
            stamp=entry.goal_info.stamp.sec*10**9+entry.goal_info.stamp.nanosec
            if stamp>=sent_stamp:statuses[bytes(entry.goal_info.goal_id.uuid).hex()]=entry.status
    subscriptions.append(node.create_subscription(GoalStatusArray,'/'+ns+'/follow_path/_action/status',action,10))
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    goal=node.create_publisher(PoseStamped,'/'+ns+'/goal_pose',10)
    stop=node.create_publisher(Bool,'/'+ns+'/target_detected',10)
    result={};motion_sent=False
    def wait(predicate,label,seconds=60):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise TimeoutError(label)
    try:
        wait(lambda:all(k in data for k in ('odom','cloud','status','truth')) and 'ready=1' in data['status'].data,'fresh EKF/sensor data')
        arrivals.clear();end=time.monotonic()+8
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        rates={key:(len(ts)-1)/(ts[-1]-ts[0]) for key,ts in arrivals.items() if len(ts)>1}
        cloud=data['cloud'];odom=data['odom'];now=time.time()
        ages={key:now-(data[key].header.stamp.sec+data[key].header.stamp.nanosec/1e9) for key in ('odom','cloud')}
        result.update(rates_wall_hz=rates,wall_stamp_age_seconds=ages,point_step=cloud.point_step,
                      point_fields=[f.name for f in cloud.fields],odom_frame=odom.header.frame_id,
                      child_frame=odom.child_frame_id,cloud_frame=cloud.header.frame_id)
        assert rates['odom']>=30, rates
        assert rates['cloud']>=8, rates
        assert all(0<=age<1 for age in ages.values()),ages
        assert odom.header.frame_id==ns+'/odom' and odom.child_frame_id==ns+'/base_link'
        assert cloud.header.frame_id==ns+'/livox_frame'
        assert cloud.point_step==12 and [f.name for f in cloud.fields]==list('xyz')
        assert [(f.offset,f.datatype,f.count) for f in cloud.fields]==[(0,7,1),(4,7,1),(8,7,1)]
        assert cloud.height==1 and len(cloud.data)==cloud.width*12==cloud.row_step
        for topic in ('mavros/odom','livox/lidar'):
            publishers=node.get_publishers_info_by_topic('/'+ns+'/'+topic)
            assert len(publishers)==1 and publishers[0].qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT
        tf_sources={entry.node_name for topic in ['/tf','/tf_static'] for entry in node.get_publishers_info_by_topic(topic)}
        result['public_tf_publishers']=sorted(tf_sources)
        assert tf_sources <= {'nav2_input_bridge','base_link_to_lidar_tf','base_to_odom_tf'},tf_sources
        if args.require_nav or args.move:
            for name in ('controller_server','straight_planner'):
                client=node.create_client(GetState,'/'+ns+'/'+name+'/get_state')
                wait(client.service_is_ready,name+' state service')
                future=client.call_async(GetState.Request());wait(future.done,name+' state reply')
                assert future.result().current_state.id==3,name
            wait(lambda:buffer.can_transform(ns+'/map',ns+'/base_link',Time()),'navigation TF')
            tf=buffer.lookup_transform(ns+'/base_link',ns+'/livox_frame',Time())
            xyz=tf.transform.translation;q=tf.transform.rotation
            assert abs(xyz.x-.18)<1e-6 and abs(xyz.z-.53)<1e-6
            assert abs(2*math.atan2(q.y,q.w)-math.radians(18))<1e-4
            assert len(node.get_publishers_info_by_topic('/'+ns+'/cmd_vel'))==1
            assert len(node.get_publishers_info_by_topic('/'+ns+'/odom'))==1
            result['navigation_active']=True
        if args.move:
            p=data['odom'].pose.pose.position;q=data['odom'].pose.pose.orientation
            truth=data['truth'].pose.pose.position;start_truth=[truth.x,truth.y]
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            message=PoseStamped();message.header.frame_id=ns+'/map'
            message.header.stamp=node.get_clock().now().to_msg()
            message.pose.position.x=p.x+math.cos(yaw);message.pose.position.y=p.y+math.sin(yaw);message.pose.orientation=q
            sent_stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec;statuses.clear()
            result['goal']=[message.pose.position.x,message.pose.position.y]
            goal.publish(message);motion_sent=True
            def arrived():
                pos=data['odom'].pose.pose.position
                return 4 in statuses.values() and math.hypot(pos.x-message.pose.position.x,pos.y-message.pose.position.y)<.35
            wait(arrived,'goal completion',75)
            pos=data['odom'].pose.pose.position;truth=data['truth'].pose.pose.position
            result.update(arrival_error_m=math.hypot(pos.x-message.pose.position.x,pos.y-message.pose.position.y),
                          truth_displacement_m=math.hypot(truth.x-start_truth[0],truth.y-start_truth[1]),
                          action_statuses=statuses,motion_success=True)
        if args.move:
            assert result['truth_displacement_m']>.3, 'No actual simulated displacement'
        result['passed']=True
    finally:
        if motion_sent:
            stop.publish(Bool(data=True))
            try:wait(lambda:'cmd' in data and abs(data['cmd'].linear.x)<.01 and abs(data['cmd'].angular.z)<.01,'stop',5)
            except TimeoutError:result['stop_verified']=False
            else:result['stop_verified']=True
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
