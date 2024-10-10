#!/home/jetson/my_venv/bin/python
import rclpy
from rclpy.node import Node
import threading
import pyaudio
from openwakeword.model import Model
import time
import numpy as np
from chatclass import SpeechRecognition
from std_msgs.msg import String

class WakeWordDetectionNode(Node):
    def __init__(self):
        super().__init__('wakeword_detection_node')
        self.publisher = self.create_publisher(String, '/wake_word_detection', 10)
        self.is_question_active = False

def detect_wakeword(wake_word_node, model, mic_stream, chat_lock, chat):
    last_detection_time = 0
    debounce_interval = 1  # Time in seconds
    CHUNK = 1280
    while rclpy.ok():
        # Skip wake word detection if a question is already active
        if wake_word_node.is_question_active:
            continue  # Don't process wake words if the question is active

        audio_data = np.frombuffer(mic_stream.read(CHUNK), dtype=np.int16)
        prediction = model.predict(audio_data)

        for mdl in model.prediction_buffer.keys():
            scores = list(model.prediction_buffer[mdl])
            if scores[-1] > 0.5:  # Threshold for wakeword detection
                current_time = time.time()
                if current_time - last_detection_time > debounce_interval:
                    last_detection_time = current_time
                    print("Wakeword Detected!")

                    # Publish wake word detection message
                    msg = String()
                    msg.data = "Wake word detected!"
                    wake_word_node.publisher.publish(msg)

                    # Set the question active flag
                    with chat_lock:
                        wake_word_node.is_question_active = True  # Set flag to True before starting question
                        threading.Thread(target=chat.ask_question, args=(wake_word_node,)).start()

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 before creating any nodes
    
    chat = SpeechRecognition()  # Initialize the chat class
    wake_word_node = WakeWordDetectionNode()  # Create the wake word detection node

    # Manually set the model path
    model_path = "/home/jetson/ros2/src/curio_one/scripts/hello_curio88.onnx"
    model_path_2 = "/home/jetson/ros2/src/curio_one/scripts/hey_curio82.onnx"
    inference_framework = 'onnx'

    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK = 1280
    audio = pyaudio.PyAudio()
    mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    # Initialize the model
    owwModel = Model(wakeword_models=[model_path,model_path_2],inference_framework=inference_framework)

    chat_lock = threading.Lock()

    # Start the wake word detection in a separate thread
    detection_thread = threading.Thread(target=detect_wakeword, args=(wake_word_node, owwModel, mic_stream, chat_lock, chat))
    detection_thread.start()

    try:
        rclpy.spin(chat)
        rclpy.spin(wake_word_node)  # Keep the ROS 2 node running
    except KeyboardInterrupt:
        wake_word_node.get_logger().info("Shutting down wake word detection node.")

    # Cleanup and shutdown
    detection_thread.join()  # Wait for the detection thread to finish
    wake_word_node.destroy_node()
    rclpy.shutdown()  # Shutdown ROS 2 after completing tasks

if __name__ == "__main__":
    main()
