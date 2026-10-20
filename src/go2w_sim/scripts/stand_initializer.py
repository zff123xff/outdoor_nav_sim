#!/usr/bin/env python3
"""Hold the 12 leg joints at the same standing pose used by ros2_control startup."""
import rclpy
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('go2w_stand_initializer')
    publisher = node.create_publisher(Float64MultiArray, '/leg_position_controller/commands', 10)
    pose = Float64MultiArray(data=[0.0, 0.67, -1.3] * 4)
    timer = node.create_timer(0.1, lambda: publisher.publish(pose))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
