#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RdePublisher(Node):
    def __init__(self):
        super().__init__('rde_publisher')
        self.publisher_ = self.create_publisher(String, 'rde', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        
        # Warning message on startup
        self.get_logger().warn('🚨 RDE Publisher node starting - will publish messages every second!')
        self.get_logger().info('RDE Publisher node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from RDE #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    rde_publisher = RdePublisher()

    try:
        rclpy.spin(rde_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rde_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
