#!/usr/bin/env python3
"""Queue atomic controller activation before unpausing Classic physics."""
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController, ListControllers
from std_srvs.srv import Empty


def main():
    rclpy.init()
    node=Node('classic_start_controllers')
    names=node.declare_parameter('controllers',['joint_state_broadcaster','leg_trajectory_controller','wheel_velocity_controller']).value
    switch=node.create_client(SwitchController,'/controller_manager/switch_controller')
    unpause=node.create_client(Empty,'/unpause_physics')
    listing=node.create_client(ListControllers,'/controller_manager/list_controllers')
    try:
        for client in (switch,unpause,listing):
            if not client.wait_for_service(timeout_sec=30):raise RuntimeError('Classic startup service unavailable: '+client.srv_name)
        req=SwitchController.Request()
        req.activate_controllers=list(names)
        req.strictness=SwitchController.Request.STRICT
        req.activate_asap=True
        req.timeout.sec=15
        pending=switch.call_async(req)
        # The switch executes in the first physics updates. Let its request reach
        # the manager while gravity is still paused, then release the world.
        end=time.monotonic()+.1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        unpaused=unpause.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node,unpaused,timeout_sec=15)
        if not unpaused.done() or unpaused.exception():raise RuntimeError('Could not unpause Classic')
        rclpy.spin_until_future_complete(node,pending,timeout_sec=25)
        if not pending.done() or pending.exception() or not pending.result().ok:raise RuntimeError('Classic controller activation failed')
        future=listing.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.exception():raise RuntimeError('Controller state verification failed')
        states={item.name:item.state for item in future.result().controller}
        if any(states.get(name)!='active' for name in names):raise RuntimeError(str(states))
        node.get_logger().info('Classic physics running; verified active controllers: '+', '.join(names))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__':main()
