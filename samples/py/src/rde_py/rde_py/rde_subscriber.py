#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RdeSubscriber(Node):
    def __init__(self):
        super().__init__('rde_subscriber')
        self.subscription = self.create_subscription(
            String,
            'rde',
            self.listener_callback,
            10)
        self.get_logger().info('RDE Subscriber node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    rde_subscriber = RdeSubscriber()

    try:
        rclpy.spin(rde_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rde_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
