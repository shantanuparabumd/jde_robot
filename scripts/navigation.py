#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

def publish_initial_pose(initial_pose_publisher):
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

    initial_pose_publisher.publish(initial_pose_msg)

def publish_goal_pose(goal_pose_publisher):
    goal_pose_msg = PoseStamped()
    # Set the goal pose here
    goal_pose_msg.pose.position.x = 1.5
    goal_pose_msg.pose.position.y = -1.5
    goal_pose_msg.pose.position.z = 0.0
    goal_pose_msg.pose.orientation.x = 0.0
    goal_pose_msg.pose.orientation.y = 0.0
    goal_pose_msg.pose.orientation.z = 0.0
    goal_pose_msg.pose.orientation.w = 1.0
    goal_pose_msg.header.frame_id = 'map'

    goal_pose_publisher.publish(goal_pose_msg)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('pose_publisher')

    initial_pose_publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    goal_pose_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)

    # Publish initial pose
    # publish_initial_pose(initial_pose_publisher)
    # node.get_logger().info('Published initial pose')

    # Publish goal pose
    publish_goal_pose(goal_pose_publisher)
    node.get_logger().info('Published goal pose')

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
