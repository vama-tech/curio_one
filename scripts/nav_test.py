#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import tf_transformations

class PoseLoggerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.odom_callback,
            10
        )
        self.odom_pose = None
        self.goal_pose = None
        self.navigator = None
        self.initial_pose_set = True  # Add flag to track if initial pose is set

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        
        if self.odom_pose and not self.initial_pose_set:
            # Set initial pose as the current robot pose
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            initial_pose.pose = self.odom_pose
            
            # Set this as the initial pose in the navigator
            self.navigator.setInitialPose(initial_pose)
            self.initial_pose_set = True  # Mark the flag to prevent resetting the initial pose
            self.get_logger().info(f"Initial Pose Set: {initial_pose.pose.position.x}, {initial_pose.pose.position.y}")
        
        if self.goal_pose:
            deviation = self.calculate_deviation(self.odom_pose, self.goal_pose)
            self.get_logger().info(f"Deviation from goal: {deviation:.2f} meters")

            # Calculate angular deviation
            angular_deviation = self.calculate_angular_deviation(self.odom_pose, self.goal_pose)
            self.get_logger().info(f"Angular Deviation from goal: {angular_deviation:.2f} degree")

    def calculate_deviation(self, odom_pose, goal_pose):
        dx = goal_pose.position.x - odom_pose.position.x
        dy = goal_pose.position.y - odom_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def calculate_angular_deviation(self, odom_pose, goal_pose):
        # Extract the orientations (quaternions)
        odom_quat = [
            odom_pose.orientation.x,
            odom_pose.orientation.y,
            odom_pose.orientation.z,
            odom_pose.orientation.w
        ]
        goal_quat = [
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w
        ]

        # Convert quaternions to Euler angles (roll, pitch, yaw)
        _, _, odom_yaw = tf_transformations.euler_from_quaternion(odom_quat)
        _, _, goal_yaw = tf_transformations.euler_from_quaternion(goal_quat)

        # Calculate angular deviation
        angular_deviation = goal_yaw - odom_yaw
        angle = (angular_deviation * 180 / math.pi)
        return angle

def main():
    rclpy.init()
    
    # Initialize the navigator
    navigator = BasicNavigator()

    # Create the PoseLoggerNode
    pose_logger_node = PoseLoggerNode()
    pose_logger_node.navigator = navigator

    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()

    # Set the goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -1.09949
    goal_pose.pose.position.y = 4.88543
    goal_pose.pose.orientation.w = 0.272163
    goal_pose.pose.orientation.z = 0.962251

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
