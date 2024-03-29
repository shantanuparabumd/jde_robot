#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher_jde')

        # Publisher to publish goal pose
        self.goal_publisher = self.create_publisher(String, '/jde_chat', 10)

        # Create a wall timer with a callback function
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello! ROS2 is fun'

        self.goal_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
