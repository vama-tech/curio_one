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


# Initialize the TTS engine
engine = pyttsx3.init()

# Initialize Mediapipe face mesh detection
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=5, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)


class WakeOnFaceDetect(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.get_logger().info("wake on face Detection Started")

        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
        )
        # Subscription to the image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Replace with the actual topic name
            self.image_callback,
            qos_profile
        )


        # Initialize necessary variables
        self.bridge = CvBridge()
        self.person_start_time = None
        self.warning_spoken = False
        self.delay_set = 5  

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # if image:
        #     self.get_logger().info("Recieved Image Data")
        # else:
        #     self.get_logger().info("not recieved Image Data")

        start = time.time()

        # Flip the image for selfie view
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # Process image with Mediapipe face mesh
        results = face_mesh.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.multi_face_landmarks:
            # Check the time since a person was first detected
            if self.person_start_time is None:
                self.person_start_time = time.time()

            # If face has been detected for more than `delay_set` seconds, trigger warning
            elif not self.warning_spoken and time.time() - self.person_start_time > self.delay_set:
                self.warning_spoken = True
                # threading.Thread(target=say_warning).start()

                self.get_logger().info("Person Detected")
        else:
            self.get_logger().info("No Person Detected")
            self.warning_spoken = False  # Reset if no person detected
            self.person_start_time = None

        # Show the image (can be disabled if not needed)
        cv2.imshow('Head Pose Detection', image)
        cv2.waitKey(1)  # Required for the OpenCV window to refresh


def main(args=None):
    rclpy.init(args=args)
    node = WakeOnFaceDetect()

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
