#!/home/jetson/my_venv/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import mediapipe as mp
import time
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from chatclass import SpeechRecognition  
import threading
import playsound

class Integration(Node):
    def __init__(self):
        super().__init__('head_pose_estimation')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.audio_playing_event = threading.Event()
        self.camera_publish = self.create_publisher(String, "/face_data", 10)
    

        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=4, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)

        self.bridge = CvBridge()
        self.get_logger().info("Head pose estimation node initialized.")


        self.person_looking_time = None
        self.last_detected_face = time.time()
        self.is_recording = False
        self.wake_word_active = False

        
        self.speech_recognition = SpeechRecognition()

    def publish_data(self,msg):
        message = String()
        message.data = msg
        self.camera_publish.publish(message)


    def play_audio_in_thread(self, audio_path):
        self.audio_playing_event.clear()  
        audio_thread = threading.Thread(target=self._play_audio, args=(audio_path,), daemon=True)
        audio_thread.start()

    def _play_audio(self, audio_path):
        playsound.playsound(audio_path)
        self.audio_playing_event.set() 


    def image_callback(self, msg):

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            start = time.time()
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = self.face_mesh.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            img_h, img_w, img_c = image.shape

            if results.multi_face_landmarks:
                self.last_detected_face = time.time()
                for face_idx, face_landmarks in enumerate(results.multi_face_landmarks):
                    face_2d = []
                    face_3d = []

                    for idx, lm in enumerate(face_landmarks.landmark):
                        if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                            if idx == 1:
                                nose_2d = (lm.x * img_w, lm.y * img_h)
                                nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)

                            x, y = int(lm.x * img_w), int(lm.y * img_h)
                            face_2d.append([x, y])
                            face_3d.append([x, y, lm.z])

                    face_2d = np.array(face_2d, dtype=np.float64)
                    face_3d = np.array(face_3d, dtype=np.float64)

                    focal_length = 1 * img_w
                    cam_matrix = np.array([[focal_length, 0, img_h / 2],
                                           [0, focal_length, img_w / 2],
                                           [0, 0, 1]])
                    distortion_matrix = np.zeros((4, 1), dtype=np.float64)

                    success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)
                    rmat, jac = cv2.Rodrigues(rotation_vec)
                    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
                    x = angles[0] * 360
                    y = angles[1] * 360
                    z = angles[2] * 360

                    if x < 20: 
                        if self.person_looking_time is None:
                            self.person_looking_time = time.time()

                        elif time.time() - self.person_looking_time > 5.0 and not self.is_recording:
                            self.publish_data("face detected")
                         
                            print("Looking into camera for 5 seconds. Start recording.")
                            self.play_audio_in_thread("/home/jetson/ros2/src/curio_one/scripts/greeting.mp3")
                            self.audio_playing_event.wait()

                            self.speech_recognition.ask_question()  
                            self.is_recording = True
                            self.wake_word_active = False 
                            print("Recording done. Waiting for wake word detection.")
                            
                    else:
                        self.person_looking_time = None

                    text = f"Face {face_idx + 1}: Looking into camera" if x < 20 else f"Face {face_idx + 1}: Looking elsewhere"
                   

                    
                    nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)
                    p1 = (int(nose_2d[0]), int(nose_2d[1]))
                    p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))
                    cv2.line(image, p1, p2, (255, 0, 0), 3)
                    self.mp_drawing.draw_landmarks(image=image, landmark_list=face_landmarks,
                                                   connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                                                   landmark_drawing_spec=self.drawing_spec,
                                                   connection_drawing_spec=self.drawing_spec)

            else:
                if time.time() - self.last_detected_face > 30:
                    self.publish_data("reset")
                    print("No face detected for 30 seconds. Resetting system.")

                    self.is_recording = False  
                    self.wake_word_active = False  
                    self.person_looking_time = None

            end = time.time()
            totalTime = end - start
            fps = 1 / totalTime
            cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
            cv2.imshow('Head Pose Detection', image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Integration()
    chat_node = SpeechRecognition()

    try:
        rclpy.spin(node)
        rclpy.spin(chat_node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
