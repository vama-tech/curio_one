#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import tf_transformations
import playsound

goal_poses = {
    'home': [-2.0049, 9.576607, 0.297930, 0.954587],
    'solar': [2.06911, 8.87763, 0.8418692, 0.539681],
    'wind': [-0.564153, 6.65056, 0.844142, 0.53612]
}

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

    for goal_key in goal_poses:
        # Set the current goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()

        # Get the goal coordinates and orientation from goal_poses dictionary
        selected_goal = goal_poses[goal_key]

        goal_pose.pose.position.x = selected_goal[0]
        goal_pose.pose.position.y = selected_goal[1]
        goal_pose.pose.orientation.w = selected_goal[2]
        goal_pose.pose.orientation.z = selected_goal[3]

        pose_logger_node.goal_pose = goal_pose.pose
        
        # Start the navigation to the current goal
        navigator.goToPose(goal_pose)
        # Log that we're heading to the next goal
        print(f"Navigating to {goal_key}: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")

        # After all goals, navigate back to the home position
        return_base = goal_poses['home']
        # home_goal = PoseStamped()
        # home_goal.header.frame_id = 'map'
        # home_goal.header.stamp = navigator.get_clock().now().to_msg()
        # home_goal.pose.position.x = return_base[0]
        # home_goal.pose.position.y = return_base[1]
        # home_goal.pose.orientation.w = return_base[2]
        # home_goal.pose.orientation.z = return_base[3]

        # # Navigate back to the home position
        # print("Navigating back to the home position...")
        # navigator.goToPose(home_goal)
        # Run the node and wait for the goal to be reached
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(f"Estimated time of arrival: {feedback.estimated_time_remaining.sec} seconds.")
            rclpy.spin_once(pose_logger_node, timeout_sec=1)

        # Check the result and provide feedback
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Successfully reached {goal_key}!")
            playsound.playsound('/home/jetson/ros2/src/curio_one/scripts/goal.mp3')
        elif result == TaskResult.CANCELED:
            print(f"Navigation to {goal_key} was canceled!")
        elif result == TaskResult.FAILED:
            print(f"Failed to reach {goal_key}!")
            playsound.playsound('/home/jetson/ros2/src/curio_one/scripts/base.mp3')
            navigator.goToPose(return_base)
        else:
            print(f"{goal_key} has an invalid return status!")

        # Wait for a moment before moving to the next goal
        print(f"Waiting before moving to the next goal...")
        rclpy.spin_once(pose_logger_node, timeout_sec=2)  # Allow the system to settle before next goal

    # Clean up after reaching all goals
    pose_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
