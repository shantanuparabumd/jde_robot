#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Publisher to publish goal pose
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Publisher to publish initial pose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Subscriber to monitor odometry
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # List of waypoints to navigate to
        self.waypoints = [
            (0.0, 0.0),     # Initial pose waypoint
            (-4.0, 1.0),           # Waypoint 1 (x, y)
            (3.0, 3.0),           # Goal pose waypoint
            (-4.0, 7.0) ,
            (3.0, 7.0)# Return to initial pose waypoint (optional
        ]

        # Index to track current waypoint
        self.current_waypoint_index = 0
        
        self.publish_initial_pose()
        

    def publish_initial_pose(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        # Set the initial pose here
        initial_pose_msg.pose.pose.position.x = self.waypoints[0][0]
        initial_pose_msg.pose.pose.position.y = self.waypoints[0][1]
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0
        initial_pose_msg.header.frame_id = 'map'

        self.initial_pose_publisher.publish(initial_pose_msg)

    def odom_callback(self, msg):
        current_pose = msg.pose.pose.position
        goal_pose = self.waypoints[self.current_waypoint_index]
        distance_to_goal = math.sqrt((goal_pose[0] - current_pose.x) ** 2 + (goal_pose[1] - current_pose.y) ** 2)
        # self.get_logger().info("Distance to goal: " + str(distance_to_goal) + " meters")
        
            
        # Check if the robot is close enough to the current waypoint
        if distance_to_goal < 0.5:  # Adjust the threshold as needed
            if self.current_waypoint_index < len(self.waypoints)-1:
                self.current_waypoint_index += 1
                self.navigate_to_waypoint(self.waypoints[self.current_waypoint_index])
                self.get_logger().info("Moving to next waypoint!")
            else:
                self.get_logger().info("Reached all waypoints!")
                self.destroy_node()
                rclpy.shutdown()

    def navigate_to_waypoint(self, waypoint):
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = waypoint[0]
        goal_msg.pose.position.y = waypoint[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        goal_msg.header.frame_id = 'map'  # Assuming the waypoints are in the map frame

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Navigating to waypoint: {waypoint}")

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    
    try:
        rclpy.spin(waypoint_navigator)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        waypoint_navigator.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        waypoint_navigator.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()

if __name__ == '__main__':
    main()