#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import math


class InitPosePub(Node):
    def __init__(self):
        super().__init__('init_pose_pblisher')

        
        # Publisher to publish initial pose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.publish_initial_pose()
        

    def publish_initial_pose(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        # Set the initial pose here
        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0
        initial_pose_msg.header.frame_id = 'map'

        self.initial_pose_publisher.publish(initial_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = InitPosePub()
    
    # Cleanly destroy the node instance
    pose_publisher.destroy_node()
    # Shut down the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()