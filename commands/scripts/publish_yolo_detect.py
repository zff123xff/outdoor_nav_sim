#!/usr/bin/env python3
"""Publish a detection event through the existing target_detected interface."""
import argparse
import json
import math
import re
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--clear', action='store_true', help='publish false (does not resume navigation)')
    parser.add_argument('--timeout', type=float, default=10.)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    ns = args.namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        parser.error('namespace must be one ROS name')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    topic = '/'+ns+'/target_detected'
    if args.dry_run:
        print(json.dumps(dict(topic=topic, data=not args.clear), indent=2))
        return
    import rclpy
    from std_msgs.msg import Bool
    rclpy.init()
    node = rclpy.create_node('yolo_detect_sender')
    detect = node.create_publisher(Bool, topic, 10)
    try:
        deadline = time.monotonic()+args.timeout
        def endpoints_ready():
            endpoints = node.get_subscriptions_info_by_topic(topic)
            bridge_ready = any(e.node_name == 'nav2_input_bridge' and e.node_namespace == '/'+ns for e in endpoints)
            return bridge_ready and detect.get_subscription_count() > 0
        while not endpoints_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError('target_detected input bridge subscriber not matched; check namespace and ROS domain')
            rclpy.spin_once(node, timeout_sec=.05)
        end = time.monotonic()+.6
        next_publish = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= next_publish:
                detect.publish(Bool(data=not args.clear))
                next_publish = time.monotonic()+.1
            rclpy.spin_once(node, timeout_sec=.025)
        print(f'Published {topic}={not args.clear}; '+
              ('detection false sent; navigation does not resume automatically.' if args.clear else
               'navigation cancellation requested through target_detected; no persistent velocity lock.'))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
