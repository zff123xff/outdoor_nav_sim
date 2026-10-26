#!/usr/bin/env python3
"""Broadcast the complete fused pose, including measured altitude, to TF."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def odometry_transform(message):
    pose = message.pose.pose
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite fused pose')
    transform = TransformStamped()
    transform.header = message.header
    transform.child_frame_id = message.child_frame_id
    transform.transform.translation.x = pose.position.x
    transform.transform.translation.y = pose.position.y
    transform.transform.translation.z = pose.position.z
    transform.transform.rotation = pose.orientation
    return transform


class FusedOdometryTF(Node):
    def __init__(self):
        super().__init__('fused_odometry_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, 'mavros/odom', self.on_odometry, qos_profile_sensor_data)

    def on_odometry(self, message):
        try:
            transform = odometry_transform(message)
        except ValueError:
            return
        if not transform.header.frame_id or not transform.child_frame_id:
            return
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = FusedOdometryTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
