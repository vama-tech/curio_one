#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from control_msgs.msg import DynamicJointState # Import your custom message type
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.get_logger().info("Started LaserScan Node")
        self.publisher_ = self.create_publisher(LaserScan, '/range_6', 10)
        self.subscription_ = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription_  # prevent unused variable warning

    def dynamic_joint_states_callback(self, msg):
        for interface_value in msg.interface_values:
            if 'Sensor6' in interface_value.interface_names:
                sensor_value = interface_value.values[5]  # Assuming only one value for simplicity
                scan_msg = LaserScan()
                scan_msg.header.stamp = self.get_clock().now().to_msg()
                scan_msg.header.frame_id = 'tof_6'  # Set appropriate frame ID
                scan_msg.angle_min = -1.57  # Set appropriate min angle (e.g., -90 degrees)
                scan_msg.angle_max = 1.57  # Set appropriate max angle (e.g., 90 degrees)
                scan_msg.angle_increment = 3.14 / 90  # Set appropriate angle increment (e.g., 1 degree)
                scan_msg.time_increment = 0.0  # Time between measurements
                scan_msg.scan_time = 0.1  # Time between scans
                scan_msg.range_min = 0.00  # Set appropriate min range
                scan_msg.range_max = 100.0  # Set appropriate max range

                num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
                ranges = [float('inf')] * num_readings  # Initialize ranges with 'inf'
                angle_index = num_readings // 2  # Assuming the sensor_value corresponds to the center angle
                ranges[angle_index] = sensor_value / 1000.0

                scan_msg.ranges = ranges
                scan_msg.intensities = [0.0] * num_readings  # Optional: Set intensities to zero

                self.publisher_.publish(scan_msg)

                # Publish transform
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'chassis'  # Set appropriate parent frame
                transform.child_frame_id = 'tof_joint_6'  # Set appropriate child frame
                transform.transform.translation.x = 0.192  # Set appropriate translation
                transform.transform.translation.y = -0.120
                transform.transform.translation.z = 0.580
                transform.transform.rotation.x = 0.0  # Set appropriate rotation
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Range
# from control_msgs.msg import DynamicJointState # Import your custom message type
# import tf2_ros
# from geometry_msgs.msg import TransformStamped

# class SensorPublisher(Node):
#     def __init__(self):
#         super().__init__('sensor_publisher')
#         self.get_logger().info("Started Range 2 Node")
#         self.publisher_ = self.create_publisher(Range, '/range_1', 10)
#         self.subscription_ = self.create_subscription(
#             DynamicJointState,
#             '/dynamic_joint_states',
#             self.dynamic_joint_states_callback,
#             10)
#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
#         self.subscription_  # prevent unused variable warning

#     def dynamic_joint_states_callback(self, msg):
#         for interface_value in msg.interface_values:
            
#             if 'Sensor2' in interface_value.interface_names:
#                 sensor_value = interface_value.values[1]  # Assuming only one value for simplicity
#                 range_msg = Range()
#                 range_msg.header.stamp = self.get_clock().now().to_msg()
#                 range_msg.header.frame_id = 'tof_1'  # Set appropriate frame ID
#                 range_msg.radiation_type = Range.INFRARED  # Set appropriate radiation type
#                 range_msg.field_of_view = 0.3  # Set appropriate FOV
#                 range_msg.min_range = 0.03  # Set appropriate min range
#                 range_msg.max_range = 6.0  # Set appropriate max range
#                 range_msg.range = sensor_value / 1000.0
#                 self.publisher_.publish(range_msg)

#                 # Publish transform
#                 transform = TransformStamped()
#                 transform.header.stamp = self.get_clock().now().to_msg()
#                 transform.header.frame_id = 'base_link'  # Set appropriate parent frame
#                 transform.child_frame_id = 'tof_joint_1'  # Set appropriate child frame
#                 transform.transform.translation.x = 0.192  # Set appropriate translation
#                 transform.transform.translation.y = 0.148
#                 transform.transform.translation.z = 0.06
#                 transform.transform.rotation.x = 0.0  # Set appropriate rotation
#                 transform.transform.rotation.y = 0.0
#                 transform.transform.rotation.z = 0.0
#                 transform.transform.rotation.w = 1.0
#                 self.tf_broadcaster.sendTransform(transform)

# def main(args=None):
#     rclpy.init(args=args)
#     sensor_publisher = SensorPublisher()
#     rclpy.spin(sensor_publisher)
#     sensor_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

