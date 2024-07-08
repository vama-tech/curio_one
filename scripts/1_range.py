#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from control_msgs.msg import DynamicJointState # Import your custom message type
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.get_logger().info("Started Range 2 Node")
        self.publisher_ = self.create_publisher(Range, '/range_1', 10)
        self.subscription_ = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription_  # prevent unused variable warning

    def dynamic_joint_states_callback(self, msg):
        for interface_value in msg.interface_values:
            
            if 'Sensor1' in interface_value.interface_names:
                sensor_value = interface_value.values[1]  # Assuming only one value for simplicity
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = 'tof_1'  # Set appropriate frame ID
                range_msg.radiation_type = Range.INFRARED  # Set appropriate radiation type
                range_msg.field_of_view = 0.1  # Set appropriate FOV
                range_msg.min_range = 0.03  # Set appropriate min range
                range_msg.max_range = 100.0  # Set appropriate max range
                range_msg.range = sensor_value / 500.0
                self.publisher_.publish(range_msg)

                # Publish transform
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'chassis'  # Set appropriate parent frame
                transform.child_frame_id = 'tof_joint_1'  # Set appropriate child frame
                transform.transform.translation.x = 0.192  # Set appropriate translation
                transform.transform.translation.y = 0.148
                transform.transform.translation.z = 0.06
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
