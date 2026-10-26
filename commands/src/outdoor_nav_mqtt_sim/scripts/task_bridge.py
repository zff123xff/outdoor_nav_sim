#!/usr/bin/env python3
"""Use the hardware task parser without publishing its mock FCU status."""
import rclpy
from multi_task_bridge.node import MultiTaskBridge


class SimulationTaskBridge(MultiTaskBridge):
    def __init__(self):
        super().__init__()
        self.destroy_timer(self.timer)
        self.get_logger().info('Hardware MultiTask parser active; mock MAVROS state timer disabled')


def main():
    rclpy.init()
    node = SimulationTaskBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
