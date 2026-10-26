#!/usr/bin/env python3
"""Publish one GPS target to the selected navigation input."""
import argparse
import json
import math
from pathlib import Path
import re
import time

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--topic', choices=['gp_goal', 'uav/target_gps'], default='gp_goal')
    parser.add_argument('--file', type=Path)
    for key in ('latitude', 'longitude', 'altitude'):
        parser.add_argument('--'+key, type=float)
    parser.add_argument('--timeout', type=float, default=15.)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--use-sim-time', action='store_true', help='Legacy Classic mode only: require /clock')
    args = parser.parse_args()
    ns = args.namespace.strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        parser.error('namespace must be one ROS name')
    values = [args.latitude, args.longitude, args.altitude]
    if args.file is not None and any(value is not None for value in values):
        parser.error('use --file or explicit GPS coordinates, not both')
    if any(value is not None for value in values):
        if any(value is None for value in values):
            parser.error('latitude, longitude and altitude are all required')
        payload = dict(zip(('latitude', 'longitude', 'altitude'), values))
    else:
        path = args.file or Path(__file__).resolve().parents[1]/'examples/gps_goal.yaml'
        payload = yaml.safe_load(path.read_text())
    lat, lon, alt = [float(payload[key]) for key in ('latitude', 'longitude', 'altitude')]
    if not all(math.isfinite(value) for value in (lat, lon, alt)) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
        parser.error('invalid GPS coordinates')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('timeout must be finite and positive')
    topic = '/'+ns+'/'+args.topic
    if args.dry_run:
        print(json.dumps(dict(topic=topic, latitude=lat, longitude=lon, altitude=alt), indent=2))
        return
    import rclpy
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import NavSatFix
    rclpy.init()
    node = rclpy.create_node('gps_target_sender', parameter_overrides=[Parameter('use_sim_time', value=args.use_sim_time)])
    publisher = node.create_publisher(NavSatFix, topic, 10)
    try:
        deadline = time.monotonic()+args.timeout
        def endpoints_ready():
            endpoints = node.get_subscriptions_info_by_topic(topic)
            bridge = any(e.node_name == 'nav2_input_bridge' and e.node_namespace == '/'+ns for e in endpoints)
            return bridge and publisher.get_subscription_count() >= len(endpoints)
        while not endpoints_ready() or node.get_clock().now().nanoseconds == 0:
            if time.monotonic() > deadline:
                raise TimeoutError('No matched target subscriber or selected clock; start simulation and navigation first')
            rclpy.spin_once(node, timeout_sec=.1)
        message = NavSatFix()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'gps'
        message.status.status = 0
        message.latitude, message.longitude, message.altitude = lat, lon, alt
        publisher.publish(message)
        # Keep the writer alive for delivery; publication is not arrival.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=.1)
        print(f'Published once to {topic}: {lat}, {lon}, {alt}; this is not an arrival confirmation.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
