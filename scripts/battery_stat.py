#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from control_msgs.msg import DynamicJointState
import tf2_ros
from geometry_msgs.msg import TransformStamped

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.get_logger().info("Started Battery Node")
        self.publisher_ = self.create_publisher(BatteryState, '/battery_stat', 10)
        self.subscription_ = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.dynamic_joint_states_callback,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription_  # prevent unused variable warning

    def dynamic_joint_states_callback(self, msg):
        battery_msg = BatteryState()
        current_value = None
        percentage_value = None
        voltage_value = None

        for interface_value in msg.interface_values:
            if 'voltage' in interface_value.interface_names:
                voltage_index = interface_value.interface_names.index('voltage')
                voltage_value = interface_value.values[voltage_index]
            if 'current' in interface_value.interface_names:
                current_index = interface_value.interface_names.index('current')
                current_value = interface_value.values[current_index]
            if 'percentage' in interface_value.interface_names:
                percentage_index = interface_value.interface_names.index('percentage')
                percentage_value = interface_value.values[percentage_index]

        # Assign the values to BatteryState message
        battery_msg.voltage = voltage_value if voltage_value is not None else 0.0
        battery_msg.current = current_value if current_value is not None else 0.0
        battery_msg.percentage = percentage_value if percentage_value is not None else 0.0

        # Fill in the BatteryState header information
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = 'battery_sensor'

        self.publisher_.publish(battery_msg)
        # self.get_logger().info(f"Published Battery Status: Voltage={battery_msg.voltage}, Current={battery_msg.current}, Percentage={battery_msg.percentage}")

def main(args=None):
    rclpy.init(args=args)
    battery_publisher = BatteryPublisher()
    rclpy.spin(battery_publisher)
    battery_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
