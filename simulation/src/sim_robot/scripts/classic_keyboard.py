#!/usr/bin/env python3
"""Terminal teleop for the Classic simulation; idle input does not mask navigation."""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Run classic_keyboard.py in an interactive terminal')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('classic_keyboard')
    linear=float(node.declare_parameter('linear_speed',.3).value)
    lateral=float(node.declare_parameter('lateral_speed',.1).value)
    angular=float(node.declare_parameter('angular_speed',.5).value)
    if not all(math.isfinite(v) and v>0 for v in (linear,lateral,angular)):
        raise ValueError('Keyboard speeds must be finite and positive')
    pub=node.create_publisher(Twist,'cmd_vel_remote',10)
    bindings={'w':(linear,0.,0.),'s':(-linear,0.,0.),
              'a':(0.,0.,angular),'d':(0.,0.,-angular),
              'q':(0.,lateral,0.),'e':(0.,-lateral,0.),' ':(0.,0.,0.)}
    saved=termios.tcgetattr(sys.stdin.fileno())
    print('W/S 前后，A/D 转向，Q/E 横移，空格停止，Ctrl+C 退出。\n按住按键可连续输入；输入间隔超过 0.25 秒自动停机。',flush=True)
    last=-math.inf;command=(0.,0.,0.);active=False
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable,_,_=select.select([sys.stdin],[],[],.05)
            if readable:
                key=sys.stdin.read(1).lower()
                if key=='\x03':break
                if key in bindings:
                    command=bindings[key];last=time.monotonic()
            fresh=time.monotonic()-last<.25
            if fresh or active:
                message=Twist()
                if fresh:message.linear.x,message.linear.y,message.angular.z=command
                pub.publish(message)
            active=fresh
            rclpy.spin_once(node,timeout_sec=0.)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,saved)
        if rclpy.ok():
            for _ in range(3):
                pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
