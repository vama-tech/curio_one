#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32, String
import json
import os
import math
import tf_transformations
import tf2_ros

class RobotPosition(Node):
    def __init__(self):
        super().__init__("robot_position")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.main_pos = {}
        self.odom_pose = None
        self.goal_pose = None
        self.navigator = None
        self.file_path = "/home/jetson/ros2/src/curio_one/scripts/location.txt"

    def load_positions(self):
        if os.path.exists(self.file_path):
            with open(self.file_path, "r") as f:
                try:
                    self.main_pos = json.load(f)
                except json.JSONDecodeError:
                    self.main_pos = {}

    def save_positions(self):
        with open(self.file_path, "w") as f:
            json.dump(self.main_pos, f, indent=4)

    def calculate_deviation(self, odom_pose, goal_pose):
        """Calculate the Euclidean distance between the current position and the goal pose."""
        dx = odom_pose['x'] - goal_pose['pos']['x']
        dy = odom_pose['y'] - goal_pose['pos']['y']
        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance

    def calculate_angular_deviation(self, odom_pose, goal_pose):
        """Calculate the angular deviation between the current orientation and the goal orientation."""
        odom_quat = [
            odom_pose['orient']['x'],
            odom_pose['orient']['y'],
            odom_pose['orient']['z'],
            odom_pose['orient']['w']
        ]
        goal_quat = [
            goal_pose['orient']['x'],
            goal_pose['orient']['y'],
            goal_pose['orient']['z'],
            goal_pose['orient']['w']
        ]

        # Convert quaternions to Euler angles (yaw) for simplicity
        _, _, odom_yaw = tf_transformations.euler_from_quaternion(odom_quat)
        _, _, goal_yaw = tf_transformations.euler_from_quaternion(goal_quat)

        angular_deviation = math.degrees(abs(odom_yaw - goal_yaw))
        return angular_deviation

    def timer_callback(self):
        try:
            # Get the robot's current position and orientation
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Update self.odom_pose with current transform
            self.odom_pose = {
                'x': trans.transform.translation.x,
                'y': trans.transform.translation.y,
                'orient': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w
                }
            }

            # Ensure goal_pose is valid before calculating deviation
            if self.goal_pose is not None:
                # Calculate deviations
                linear_deviation = self.calculate_deviation(self.odom_pose, self.goal_pose)
                angular_deviation = self.calculate_angular_deviation(self.odom_pose, self.goal_pose)

                # Print deviations
                self.get_logger().info(f'Linear deviation from goal: {linear_deviation}')
                self.get_logger().info(f'Angular deviation from goal: {angular_deviation}')

        except tf2_ros.LookupException:
            self.get_logger().warn('Transform not available')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'ExtrapolationException: {str(e)}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f'ConnectivityException: {str(e)}')
        except tf2_ros.TransformException as e:
            self.get_logger().error(f'TransformException: {str(e)}')


class LocationNavigator(Node):
    def __init__(self):
        super().__init__('location_navigator')

        # Create a subscriber to listen to the location select topic
        self.subscription = self.create_subscription(
            String,
            '/location_select',
            self.location_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.navigation_feedback = self.create_publisher(String, "/navigation_feedback", 10)
        # Initialize the navigation commander
        self.navigator = BasicNavigator()
        self.pose_logger_node = RobotPosition()
        self.pose_logger_node.navigator = self.navigator
        
        # Load locations from file
        self.locations = self.load_locations("/home/jetson/ros2/src/curio_one/scripts/location.txt")
        if "Home" not in self.locations:
            self.get_logger().error("Home location not found in the file.")
            rclpy.shutdown()

        # Set initial pose to "Home"
        home_location = self.locations["Home"]
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = home_location["pos_x"]
        initial_pose.pose.position.y = home_location["pos_y"]
        initial_pose.pose.orientation.z = home_location["orient_z"]
        initial_pose.pose.orientation.w = home_location["orient_w"]

        # Start navigation
        self.navigator.waitUntilNav2Active()

    def navigation_feedback_publisher(self, message):
        msg = String()
        msg.data = message
        self.navigation_feedback.publish(msg)

    def load_locations(self, file_path):
        if os.path.exists(file_path):
            with open(file_path, "r") as f:
                try:
                    return json.load(f)
                except json.JSONDecodeError:
                    self.get_logger().error("Error decoding JSON file.")
                    return {}
        return {}

    def location_callback(self, msg: String):
        selected_location = msg.data

        if selected_location == "shutdown":
            self.get_logger().info("Shutting down...")
            rclpy.shutdown()
            return

        if selected_location not in self.locations:
            self.get_logger().warn(f"{selected_location} location not found in the file.")
            return

        # Set the goal pose
        location = self.locations[selected_location]
        self.pose_logger_node.goal_pose = {
            "pos": {
                "x": location["pos_x"],
                "y": location["pos_y"]
            },
            "orient": {
                "x": 0.0, "y": 0.0, "z": location["orient_z"], "w": location["orient_w"]
            }
        }

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = location["pos_x"]
        goal_pose.pose.position.y = location["pos_y"]
        goal_pose.pose.orientation.z = location["orient_z"]
        goal_pose.pose.orientation.w = location["orient_w"]

        # Navigate to the goal pose
        self.navigator.goToPose(goal_pose)

        # Monitor navigation progress
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        # Check the result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Arrived at {selected_location}")
            self.navigation_feedback_publisher(f"Arrived at {selected_location}")
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f"Navigation to {selected_location} canceled.")
            self.navigation_feedback_publisher(f"Navigation to {selected_location} canceled.")
        elif result == TaskResult.FAILED:
            self.get_logger().info(f"Navigation to {selected_location} failed.")
            self.navigation_feedback_publisher(f"Navigation to {selected_location} failed.")

def main():
    rclpy.init()
    location_navigator = LocationNavigator()
    rclpy.spin(location_navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
