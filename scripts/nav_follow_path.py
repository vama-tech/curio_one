#! /usr/bin/env python3


from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


"""
Basic navigation demo to follow a given path
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.615041
    initial_pose.pose.position.y = -0.77466
    initial_pose.pose.orientation.z = -0.72663
    initial_pose.pose.orientation.w = 0.707584
    navigator.goToPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.615041
    goal_pose.pose.position.y = -3.77466
    goal_pose.pose.orientation.w = 0.707584

    # Sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose)

    # Follow path
    navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated distance remaining to goal position: ' +
                  '{0:.3f}'.format(feedback.distance_to_goal) +
                  '\nCurrent speed of the robot: ' +
                  '{0:.3f}'.format(feedback.speed))

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    # exit(0)


if __name__ == '__main__':
    main()