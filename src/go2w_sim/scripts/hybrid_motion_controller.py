#!/usr/bin/env python3
"""Classic Go2-W velocity split adapted to Gazebo Classic controller interfaces.

Reference: robotdog_nav/go2w_control/src/hybrid_motion_controller.py.
CHAMP receives (0, vy, wz); wheels receive differential (vx, wz).
This node is a drive adapter downstream of twist_mux, not an arbiter.
"""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class HybridCommand:
    """Limit whole-body commands and apply a monotonic input watchdog."""
    def __init__(self, radius=.086, separation=.3802, max_wheel_speed=30.1,
                 max_linear=.3, max_lateral=.25, max_angular=.5, timeout=.5):
        values = (radius, separation, max_wheel_speed, max_linear,
                  max_lateral, max_angular, timeout)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('Hybrid geometry, limits and timeout must be finite and positive')
        self.radius, self.separation, self.max_wheel_speed = values[:3]
        self.limits = values[3:6]
        self.timeout = timeout
        self.command = (0., 0., 0.)
        self.received_at = None

    def update(self, vx, vy, wz, now):
        if not all(math.isfinite(v) for v in (vx, vy, wz, now)):
            self.command = (0., 0., 0.)
            self.received_at = None
            return False
        self.command = tuple(max(-limit, min(limit, v))
                             for v, limit in zip((vx, vy, wz), self.limits))
        self.received_at = now
        return True

    def output(self, now):
        vx, vy, wz = self.command
        if self.received_at is None or not 0 <= now-self.received_at <= self.timeout:
            vx = vy = wz = 0.
        left = (vx-wz*self.separation/2.)/self.radius
        right = (vx+wz*self.separation/2.)/self.radius
        scale = min(1., self.max_wheel_speed/max(abs(left), abs(right), 1e-12))
        # Keep leg yaw and wheel yaw consistent if joint limits constrain the turn.
        return (0., vy, wz*scale), (left*scale, right*scale, left*scale, right*scale)


class HybridMotionController(Node):
    def __init__(self):
        super().__init__('hybrid_motion_controller')
        def param(name, default):
            return float(self.declare_parameter(name, default).value)
        self.drive = HybridCommand(
            radius=param('wheel_radius', .086),
            separation=param('wheel_separation', .3802),
            max_wheel_speed=param('max_wheel_speed', 30.1),
            max_linear=param('max_linear_speed', .3),
            max_lateral=param('max_lateral_speed', .25),
            max_angular=param('max_angular_speed', .5),
            timeout=param('cmd_vel_timeout', .5))
        self.legs = self.create_publisher(Twist, 'cmd_vel_legs', 10)
        self.wheels = self.create_publisher(Float64MultiArray, 'wheel_commands', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.receive, 10)
        # Continue sending zero during a paused simulation and on lost input.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02, self.publish_command, clock=self.wall_clock)
        self.get_logger().info('Hybrid drive: CHAMP legs + four velocity-controlled wheels; watchdog=%.2fs' % self.drive.timeout)

    def receive(self, message):
        if not self.drive.update(message.linear.x, message.linear.y,
                                 message.angular.z, time.monotonic()):
            self.get_logger().warning('Non-finite drive command rejected; stopping.')

    def publish_command(self):
        legs, wheels = self.drive.output(time.monotonic())
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = legs
        self.legs.publish(message)
        self.wheels.publish(Float64MultiArray(data=list(wheels)))

    def stop(self):
        self.drive.received_at = None
        self.publish_command()


def main():
    rclpy.init()
    node = HybridMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
