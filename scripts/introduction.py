#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time
import playsound

# Positions for intros and home
poses = {
    'home': [-1.21773, 5.03698, -0.75162, 0.659596],
    'intro_1': [-2.60573, 2.45163, 0.998533, 0.0541421],
    'intro_2': [-5.61856, 1.87079, 0.999397, 0.0347318]
}

def play_audio(file_path):
    print(f"Playing audio file: {file_path}")
    playsound.playsound(file_path, True)
    time.sleep(5)

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Set the initial pose
    home_pose = PoseStamped()
    home_pose.header.frame_id = 'map'
    home_pose.header.stamp = navigator.get_clock().now().to_msg()
    home_pose.pose.position.x = poses['home'][0]
    home_pose.pose.position.y = poses['home'][1]
    home_pose.pose.orientation.z = poses['home'][2]
    home_pose.pose.orientation.w = poses['home'][3]
    navigator.goToPose(home_pose)
    
    intro_names = ['intro_1', 'intro_2']

    for index, intro in enumerate(intro_names):
        intro_pose = PoseStamped()
        intro_pose.header.frame_id = 'map'
        intro_pose.header.stamp = navigator.get_clock().now().to_msg()
        intro_pose.pose.position.x = poses[intro][0]
        intro_pose.pose.position.y = poses[intro][1]
        intro_pose.pose.orientation.z = poses[intro][2]
        intro_pose.pose.orientation.w = poses[intro][3]
        
        print(f"Navigating to {intro}...")
        navigator.goToPose(intro_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f"Estimated time of arrival at {intro}: {eta:.0f} seconds.")
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Reached {intro}. Playing audio...")
            play_audio('/home/jetson/ros2_ws/src/curio_one/scripts/' + intro + '.mp3')
            time.sleep(10)
            if index < len(intro_names) - 1:
                play_audio('/home/jetson/ros2_ws/src/curio_one/scripts/fine.mp3')
        else:
            print(f"Failed to reach {intro}. Returning to home.")
            navigator.goToPose(home_pose)
            while not navigator.isTaskComplete():
                pass
            return

    # Return to home
    print("Returning to home position...")
    navigator.goToPose(home_pose)
    while not navigator.isTaskComplete():
        pass
    
    print("Returned to home position!")

if __name__ == '__main__':
    main()
