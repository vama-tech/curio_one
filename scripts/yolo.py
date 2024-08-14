#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
import playsound

class YoloNode(Node):
    def __init__(self):
        super().__init__("Yolo_Node")
        self.get_logger().info("Testing Yolo Model")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.ros2_wsKEEP_LAST,
            depth=10
        )
        
        self.img_subscriber = self.create_subscription(Image, "/image_raw", self.camera_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.yolo_img_publisher = self.create_publisher(Image, "/yolo_img", 10)
        
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        results = self.model(source=img)
        
        processed_img = results[0].plot()  
        
        img_to_publish = self.bridge.cv2_to_imgmsg(processed_img, "bgr8")
        
        self.yolo_img_publisher.publish(img_to_publish)
        
        self.get_logger().info("Published processed image")


        person_detected = False
        for detection in results[0].boxes:
            if detection.cls == 0:  
                person_detected = True
                break
        
        if person_detected > 2:
            self.get_logger().info("Multi Person Detected")
            playsound.playsound("/home/jetson/ros2_ws/src/curio_one/scripts/greeting.mp3", True)




def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
