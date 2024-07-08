#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from icm20948 import ICM20948
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
import math

class ICM20948CLASS(Node):
    def __init__(self):
        super().__init__("imu")

        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("i2c_address", 0x69)
        i2c_addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_addr = i2c_addr

        self.declare_parameter("frame_id", "base_link")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("child_frame_id", "chassis")
        child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        self.child_frame_id = child_frame_id

        self.declare_parameter("pub_rate", 30)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        self.imu = ICM20948(self.i2c_addr)
        self.logger.info("Connected...")

        self.imu_pub_ = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(MagneticField, "/imu/mag", 10)
        self.temp_pub_ = self.create_publisher(Temperature, "/imu/temp", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def publish_cback(self):
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()

        # Read IMU data
        mag_x, mag_y, mag_z = self.imu.read_magnetometer_data()
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.imu.read_accelerometer_gyro_data()
        temp = self.imu.read_temperature()

        # Fill IMU message
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        imu_msg.linear_acceleration.x = acc_x*9.8
        imu_msg.linear_acceleration.y = acc_y*9.8
        imu_msg.linear_acceleration.z = acc_z*9.8
        imu_msg.angular_velocity.x = gyro_x*0.0174
        imu_msg.angular_velocity.y = gyro_y*0.0174
        imu_msg.angular_velocity.z = gyro_z*0.0174

        # Calculate orientation from accelerometer data
        # roll = math.atan2(acc_y, acc_z)
        # pitch = math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2))
        # # Mx = mag_y * math.cos(roll) - mag_z * math.sin(roll)
        # # My = mag_x * math.cos(pitch) + mag_y * math.sin(roll)*math.sin(pitch) + mag_z * math.cos(roll) * math.sin(pitch)
        # # yaw = math.atan2(math.sqrt(acc_x*2 + acc_y*2), acc_z)
        # yawX = mx_norm*cos(Pitch) + my_norm*sin(Pitch)*sin(Roll) + mz_norm*sin(Pitch)*cos(Roll) ;
        # yawY = my_norm*cos(Roll) - mz_norm*sin(Roll) ;

        # yaw = math.atan2(mag_x, -mag_y)
        # # yaw = math.atan2(-My, Mx)
        # self.logger.info(f"YAW: {yaw} : {mag_x, -mag_y}")
        
        # quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        # imu_msg.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=1.0)

        # Fill MagneticField message
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        # Fill Temperature message
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.frame_id
        temp_msg.temperature = temp

        # Create and publish TF transform
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = self.frame_id
        # t.child_frame_id = self.child_frame_id
        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 0.0
        # t.transform.translation.z = 0.0
        # t.transform.rotation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Publish messages and transform
        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)
        self.temp_pub_.publish(temp_msg)
        # self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    icm20948 = ICM20948CLASS()
    rclpy.spin(icm20948)

    icm20948.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
