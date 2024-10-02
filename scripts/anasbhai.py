#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import playsound

class PoseLoggerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.odom_pose = None
        self.goal_pose = None

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        if self.goal_pose:
            deviation = self.calculate_deviation(self.odom_pose, self.goal_pose)
            self.get_logger().info(f"Deviation from goal: {deviation:.2f} meters")
            

    def calculate_deviation(self, odom_pose, goal_pose):
        dx = goal_pose.position.x - odom_pose.position.x
        dy = goal_pose.position.y - odom_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    


def main():
    rclpy.init()
    
    # Initialize the navigator
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Set the goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.60573
    goal_pose.pose.position.y = 2.45163
    goal_pose.pose.orientation.w = 0.0541421
    goal_pose.pose.orientation.z = 0.998533

    # Create the PoseLoggerNode
    pose_logger_node = PoseLoggerNode()
    pose_logger_node.goal_pose = goal_pose.pose
    
    # Start the navigation
    navigator.goToPose(goal_pose)

    # Run the node
    rclpy.spin_once(pose_logger_node, timeout_sec=1)
    
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f"Estimated time of arrival: {feedback.estimated_time_remaining.sec} seconds.")
        rclpy.spin_once(pose_logger_node, timeout_sec=1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
        print(f"Goal Pose: {goal_pose}")
        print("Playing!!!")
        playsound.playsound("/home/jetson/ros2_ws/src/curio_one/scripts/hello_anas.mp3")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    # Clean up
    pose_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
