#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import time

class DockingNavigator(Node):
    def __init__(self):
        super().__init__('docking_navigator')
        self.navigator = BasicNavigator()
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def reverse_to_dock(self):
        self.get_logger().info('Reversing to the dock...')

        linear_velocity = -0.08  # Reverse speed in meters per second
        distance_to_reverse = 0.8  # Distance to reverse in meters

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        start_time = self.navigator.get_clock().now().to_msg().sec

        while True:
            current_time = self.navigator.get_clock().now().to_msg().sec
            elapsed_time = current_time - start_time
            if elapsed_time * abs(linear_velocity) >= distance_to_reverse:
                break
            self.publisher_cmd_vel.publish(cmd_vel_msg)
            time.sleep(0.1)

        self.stop_robot()
        self.get_logger().info('Successfully reversed to the charging dock!')

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(cmd_vel_msg)

def main():
    rclpy.init()

    docking_navigator = DockingNavigator()
    navigator = docking_navigator.navigator

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Define the docking pose slightly ahead of the charging dock
    pre_dock_pose = PoseStamped()
    pre_dock_pose.header.frame_id = 'map'
    pre_dock_pose.header.stamp = navigator.get_clock().now().to_msg()
    pre_dock_pose.pose.position.x = 1.00525 # position in front of the dock
    pre_dock_pose.pose.position.y = -0.489365
    pre_dock_pose.pose.orientation.w = 0.999885
    pre_dock_pose.pose.orientation.z = -0.100801

    # Navigate to the pre-dock position
    navigator.goToPose(pre_dock_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                )
                + ' seconds.'
            )

    # Check the result of the navigation task
    result = navigator.getResult()
    if result != TaskResult.SUCCEEDED:
        print('Failed to reach the pre-dock position')
        return

    # Reverse to the dock
    docking_navigator.reverse_to_dock()

    # Check the result of the docking task
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Docking succeeded!')
    else:
        print('Docking failed!')

    # Shutdown the navigator
    rclpy.shutdown()

if __name__ == '__main__':
    main()
