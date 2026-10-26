#!/usr/bin/env python3
"""Publish a bounded body velocity through the existing twist_mux remote input."""
import argparse
import json
import math
import re
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--vx', type=float, default=0., help='forward m/s; negative is backward')
    parser.add_argument('--vy', type=float, default=0., help='left m/s; negative is right')
    parser.add_argument('--wz', type=float, default=0., help='counterclockwise rad/s')
    parser.add_argument('--duration', type=float, default=1., help='wall-clock seconds')
    parser.add_argument('--rate', type=float, default=20., help='wall-clock Hz, 10..100')
    parser.add_argument('--timeout', type=float, default=10.)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    ns = args.namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        parser.error('namespace must be one ROS name')
    if not all(math.isfinite(v) for v in (args.vx, args.vy, args.wz, args.duration, args.rate, args.timeout)):
        parser.error('all numeric arguments must be finite')
    if args.duration <= 0 or args.timeout <= 0 or not 10 <= args.rate <= 100:
        parser.error('duration/timeout must be positive; rate must be 10..100 Hz')
    if abs(args.vx) > 1.5 or abs(args.vy) > .6 or abs(args.wz) > 1.:
        parser.error('ZNY limits: |vx|<=1.5 m/s, |vy|<=0.6 m/s, |wz|<=1 rad/s')
    topic = '/'+ns+'/cmd_vel_remote'
    if args.dry_run:
        print(json.dumps(dict(topic=topic, vx=args.vx, vy=args.vy, wz=args.wz,
                              duration=args.duration, rate=args.rate), indent=2))
        return
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('manual_velocity_sender')
    publisher = node.create_publisher(Twist, topic, 10)
    cancel = node.create_publisher(Bool, '/'+ns+'/stop_planner', 10)
    detected = [False]
    subscription = node.create_subscription(Bool, '/'+ns+'/target_detected',
                                            lambda m: detected.__setitem__(0, bool(m.data)), 10)
    started = False
    try:
        deadline = time.monotonic()+args.timeout
        def mux_ready():
            endpoints = node.get_subscriptions_info_by_topic(topic)
            return (any(e.node_name == 'twist_mux' and e.node_namespace == '/'+ns for e in endpoints)
                    and publisher.get_subscription_count() >= len(endpoints))
        while not mux_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError('No cmd_vel_remote subscriber; start navigation with the remote twist_mux input')
            rclpy.spin_once(node, timeout_sec=.05)
        # Cancel an ongoing path before manual control; do not resume it on exit.
        cancel.publish(Bool(data=True))
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = args.vx, args.vy, args.wz
        deadline = time.monotonic()+args.duration
        started = True
        print(f'Publishing {topic}: vx={args.vx}, vy={args.vy}, wz={args.wz}, duration={args.duration}s', flush=True)
        next_publish = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline and not detected[0]:
            now = time.monotonic()
            if now >= next_publish:
                publisher.publish(message)
                next_publish = now+1./args.rate
            rclpy.spin_once(node, timeout_sec=max(0., min(.05, next_publish-time.monotonic())))
        if detected[0]:
            print('target_detected=true received; ending manual velocity.', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if started and rclpy.ok():
            end = time.monotonic()+.3
            while time.monotonic() < end:
                publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.025)
            print('Zero velocity sent; manual publisher stopped.', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
