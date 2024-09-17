#!/home/jetson/wake_word_detection/wake/bin/python


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import the message type for publishing
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np


class HeadPoseDetection(Node):

    def __init__(self):
        super().__init__('head_pose_detection')
        
        # Subscribe to the /image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        # chat = SpeechRecognition() 
        self.br = CvBridge()
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)
        
        # Publisher for notifying after 5 seconds of looking into the camera
        self.publisher_ = self.create_publisher(String, 'camera_notification', 10)

        # Timer variables
        self.look_timer = 0
        self.no_detection_timer = 0
        self.person_detected = False
        self.look_threshold = 5  # seconds
        self.reset_threshold = 30  # seconds
    
    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(data)
        
        # Flip the image and convert to RGB for processing
        image = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        
        # Process image for face landmarks
        results = self.face_mesh.process(image)
        
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        img_h, img_w, img_c = image.shape
        self.person_detected = False
        
        if results.multi_face_landmarks:
            self.person_detected = True  # At least one face detected

            for face_idx, face_landmarks in enumerate(results.multi_face_landmarks):
                face_2d = []
                face_3d = []
                
                # Loop through specific landmarks for head pose estimation
                for idx, lm in enumerate(face_landmarks.landmark):
                    if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                        if idx == 1:
                            nose_2d = (lm.x * img_w, lm.y * img_h)
                            nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)
                        
                        x, y = int(lm.x * img_w), int(lm.y * img_h)
                        face_2d.append([x, y])
                        face_3d.append([x, y, lm.z])
                
                # Convert face landmarks to numpy arrays
                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)
                
                focal_length = 1 * img_w
                
                # Camera matrix for solvePnP
                cam_matrix = np.array([[focal_length, 0, img_h / 2],
                                       [0, focal_length, img_w / 2],
                                       [0, 0, 1]])
                distortion_matrix = np.zeros((4, 1), dtype=np.float64)
                
                # Solve PnP for face pose
                success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)
                
                # Get rotational matrix
                rmat, jac = cv2.Rodrigues(rotation_vec)
                
                # Get pose angles
                angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
                
                x = angles[0] * 360
                y = angles[1] * 360
                z = angles[2] * 360
                
                # Determine where each face is looking
                if x < 10:
                    text = f"Face {face_idx + 1}: Looking into camera"
                    
                    # Start timer for looking into the camera
                    self.look_timer += 1/30  # Assuming 30 FPS

                    # Check if the person has been looking for the threshold time
                    if self.look_timer >= self.look_threshold:
                        self.publish_camera_message()
                else:
                    text = f"Face {face_idx + 1}: Looking elsewhere"
                    self.look_timer = 0  # Reset timer if not looking into the camera
                
                # Project the nose point into 2D space
                nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)
                
                p1 = (int(nose_2d[0]), int(nose_2d[1]))
                p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))
                
                # Draw a line representing head direction
                cv2.line(image, p1, p2, (255, 0, 0), 3)
                
                # Display the direction for each face
                cv2.putText(image, text, (20, 50 + face_idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(image, f"x: {np.round(x, 2)}", (500, 50 + face_idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image, f"y: {np.round(y, 2)}", (500, 100 + face_idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image, f"z: {np.round(z, 2)}", (500, 150 + face_idx * 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Draw face landmarks and connections for each face
                self.mp_drawing.draw_landmarks(image=image,
                                               landmark_list=face_landmarks,
                                               connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                                               landmark_drawing_spec=self.drawing_spec,
                                               connection_drawing_spec=self.drawing_spec)
        
        # Handle detection reset
        if not self.person_detected:
            self.no_detection_timer += 1/30  # Increment no detection timer
            self.look_timer = 0  # Reset look timer if no face detected
            
            if self.no_detection_timer >= self.reset_threshold:
                self.no_detection_timer = 0  # Reset the no detection timer

        else:
            self.no_detection_timer = 0  # Reset the no detection timer if a person is detected
        
        # Display the image in a window
        cv2.imshow("Head Pose Detection", image)
        cv2.waitKey(1)
    
    def publish_camera_message(self):
        msg = String()
        msg.data = "Person has been looking into the camera for 5 seconds"
        self.publisher_.publish(msg)
        self.get_logger().info("Published: 'Person has been looking into the camera for 5 seconds'")
        # Reset the look timer after publishing
        self.look_timer = 0

def main(args=None):
    rclpy.init(args=args)
    node = HeadPoseDetection()
    rclpy.spin(node)
    
    # Destroy the node and clean up
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
