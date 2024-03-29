#!/usr/bin/env python3

import rclpy

from std_msgs.msg import String

def callback(msg):
    print("Received message:", msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("subscriber_node")

    subscription = node.create_subscription(
        String,
        "jde_chat",
        callback,
        10
    )

    subscription

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

if __name__ == "__main__":
    main()
