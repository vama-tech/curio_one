#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import mediapipe as mp
import time
import pyttsx3
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist


# Initialize the TTS engine
engine = pyttsx3.init()

# Initialize Mediapipe face mesh detection
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=5, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)

# Function to handle speech in a separate thread with repeating warnings
def say_warning(node):
    while node.moving and node.warning_spoken:  # Continue while robot is moving and warning has been triggered
        engine.say("Please Move Aside")
        engine.runAndWait()
        time.sleep(2)  # Add a delay of 5 seconds between warnings
    node.warning_spoken = False

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.get_logger().info("Face Detection Started")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

     
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  
            self.image_callback,
            qos_profile
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        self.bridge = CvBridge()
        self.person_start_time = None
        self.warning_spoken = False
        self.delay_set = 1 
        self.moving = False

    def cmd_vel_callback(self, msg):
        linear_movement = msg.linear.x != 0.0 or msg.linear.y != 0.0 or msg.linear.z != 0.0
        angular_movement = msg.angular.x != 0.0 or msg.angular.y != 0.0 or msg.angular.z != 0.0

    
        if linear_movement or angular_movement:
            self.get_logger().info("Robot is moving")
            self.moving = True
        else:
            self.get_logger().info("Robot is stationary")
            self.moving = False
    def image_callback(self, msg):
     
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        start = time.time()

        
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

     
        results = face_mesh.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.multi_face_landmarks:
           
            if self.person_start_time is None:
                self.person_start_time = time.time()

            elif not self.warning_spoken and time.time() - self.person_start_time > self.delay_set and self.moving:
                self.warning_spoken = True
                threading.Thread(target=say_warning, args=(self,)).start()  # Start warning in a separate thread
                self.moving=False
            print("Person Detected")
        else:
            print("No Person Detected")
            self.warning_spoken = False  # Reset if no person detected
            self.person_start_time = None

        # Show the image (can be disabled if not needed)
        cv2.imshow('Head Pose Detection', image)
        cv2.waitKey(1)  # Required for the OpenCV window to refresh


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
