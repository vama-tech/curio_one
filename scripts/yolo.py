#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
# from gtts import gTTS as GT
import pyttsx3
import time




class YoloNode(Node):
    def __init__(self):
        super().__init__("Yolo_Node")
        self.get_logger().info("Testing Yolo Model")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.img_subscriber = self.create_subscription(Image, "/image_raw", self.camera_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist,"/cmd_vel_image",10)
        self.yolo_img_publisher = self.create_publisher(Image, "/yolo_img", 10)
        self.cmd_vel_sub = self.create_subscription(Twist,"/cmd_vel",self.cmd_vel_callback,qos_profile)
        self.moving = False
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.text = "Please Move Aside"
        self.engine = pyttsx3.init()
        self.person_time = None

    def cmd_vel_callback(self,msg):
        linear_movement = msg.linear.x != 0.0 or msg.linear.y !=0.0 or msg.linear.z !=0.0
        angular_movement = msg.angular.x !=0.0 or msg.angular.y !=0.0 or msg.angular.z !=0.0

        if linear_movement or angular_movement:
            self.get_logger().info("Robot is moving")
            self.moving = True
        else:
            self.get_logger().info("Robot is Stationary")
            self.moving = False

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
        
        if person_detected:
            if self.person_time is None:
                self.person_time = time.time()
            if time.time() - self.person_time > 5:
                self.get_logger().info("Person Detected, Please Move Aside")
                self.engine.say("Please Move Aside")
                self.engine.runAndWait()

        else:
            self.person_time = None

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
