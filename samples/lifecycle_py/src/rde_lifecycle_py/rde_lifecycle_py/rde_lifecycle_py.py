#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from lifecycle_msgs.msg import State as LifecycleState


class RdeLifecycleNode(LifecycleNode):
    """
    A simple ROS 2 lifecycle node example.
    """
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info('RdeLifecycleNode created')
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle state transition: 'configuring' -> 'inactive'
        """
        self.get_logger().info('on_configure() is called')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle state transition: 'activating' -> 'active'
        """
        self.get_logger().info('on_activate() is called')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle state transition: 'deactivating' -> 'inactive'
        """
        self.get_logger().info('on_deactivate() is called')
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle state transition: 'cleaningup' -> 'unconfigured'
        """
        self.get_logger().info('on_cleanup() is called')
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle state transition: 'shuttingdown' -> 'finalized'
        """
        self.get_logger().info('on_shutdown() is called')
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, state: State) -> TransitionCallbackReturn:
        """
        Lifecycle state transition: 'errorprocessing' -> 'unconfigured'
        """
        self.get_logger().info('on_error() is called')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    
    node = RdeLifecycleNode('rde_lifecycle_py')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
