#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time
import playsound

# Positions for exhibits and home
poses = {
    'home': [1.07151, -0.604754, -0.766041, 0.642791],
    'exhibit_1': [0.936196, -1.80679, -0.769206, 0.639001],
    'exhibit_2': [0.760496, -2.83745, -0.782108, 0.623143],
    'exhibit_3': [0.631198, -3.96674, -0.756141, 0.654409]
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
    
    exhibit_names = ['exhibit_1', 'exhibit_2', 'exhibit_3']

    for index, exhibit in enumerate(exhibit_names):
        exhibit_pose = PoseStamped()
        exhibit_pose.header.frame_id = 'map'
        exhibit_pose.header.stamp = navigator.get_clock().now().to_msg()
        exhibit_pose.pose.position.x = poses[exhibit][0]
        exhibit_pose.pose.position.y = poses[exhibit][1]
        exhibit_pose.pose.orientation.z = poses[exhibit][2]
        exhibit_pose.pose.orientation.w = poses[exhibit][3]
        
        print(f"Navigating to {exhibit}...")
        navigator.goToPose(exhibit_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f"Estimated time of arrival at {exhibit}: {eta:.0f} seconds.")
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Reached {exhibit}. Playing audio...")
            play_audio('/home/jetson/ros2_ws/src/curio_one/scripts/' + exhibit + '.mp3')
            time.sleep(10)
            if index < len(exhibit_names) - 1:
                play_audio('/home/jetson/ros2_ws/src/curio_one/scripts/next.mp3')
        else:
            print(f"Failed to reach {exhibit}. Returning to home.")
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
