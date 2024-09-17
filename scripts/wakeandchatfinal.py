#!/home/jetson/wake_word_detection/wake/bin/python

import pyaudio
import numpy as np
from openwakeword.model import Model
import time
import requests
from gtts import gTTS
from scipy.io.wavfile import write
import sounddevice as sd
from pydub import AudioSegment
import playsound
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import webrtcvad
import collections


# Define a ROS 2 node to publish the detected question
class NavigationPublisher(Node):
    def __init__(self):
        super().__init__('navigation_publisher')
        self.publisher_ = self.create_publisher(Int32, 'location_select', 10)

    def publish_command(self, command):
        msg = Int32()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published command: {command}')


def chat(navigation_publisher):
    # Initialize WebRTC VAD
    vad = webrtcvad.Vad()
    vad.set_mode(1)  # 1 is low sensitivity, can increase for more aggressive VAD

    # Function to check if audio contains speech
    def is_speech(frame, sample_rate):
        return vad.is_speech(frame, sample_rate)

    # Record audio with a delay before stopping on silence
    def record_on_speech(file_path, duration=5, fs=16000, speech_threshold=1.0, min_record_time=2.0, silence_duration=1.0):
        print("Listening for speech...")

        buffer = collections.deque(maxlen=30)  # Buffer to store audio frames
        recording = []  # Store the final recording
        speech_detected = False
        recording_start_time = None
        silence_start_time = None

        with sd.InputStream(samplerate=fs, channels=1, dtype='int16') as stream:
            while True:
                frame, _ = stream.read(int(fs / 100))  # Read 10ms of audio
                buffer.append(frame)

                # Convert audio frame to bytes for VAD
                frame_bytes = np.frombuffer(frame, np.int16).tobytes()

                if is_speech(frame_bytes, fs):
                    if not speech_detected:
                        print("Recording started...")
                        speech_detected = True
                        recording_start_time = time.time()
                        recording.extend(buffer)
                    else:
                        # Reset the silence timer if speech is detected
                        silence_start_time = None
                    
                    # Continue recording
                    recording.append(frame)

                elif speech_detected:
                    # If speech stops, start counting silence duration
                    if silence_start_time is None:
                        silence_start_time = time.time()
                    
                    # Stop recording if silence exceeds the allowed duration
                    if time.time() - silence_start_time >= silence_duration and time.time() - recording_start_time > min_record_time:
                        print("Speech ended due to silence")
                        break

        # Save the recording to a WAV file
        recording = np.concatenate(recording, axis=0)
        write(file_path, fs, recording)
        print(f"Recording saved to {file_path}")





    # POST Request Function
    def post_request(url, audio_file):
        response = requests.post(url, files=audio_file)
        return response

    # Base URL Template
    front_url = "https://1ab4-122-176-225-5.ngrok-free.app"  # ALWAYS CHECK THE NGROK URL AND UPDATE HERE

    # 'en' for English or 'bn' for Bengali
    language = "english"
    language_1 = "en"
    robot_index = 5



    temp_wav = "input.wav"
    temp_mp3 = "temp.mp3"
    # Record audio and save it as a WAV file with a delay before stopping
    record_on_speech(temp_wav, silence_duration=0.5)



    # Convert the WAV file to MP3
    audio = AudioSegment.from_wav(temp_wav)
    # audio.export(temp_mp3, format="mp3")

    # Keep the audio file name as generic as possible
    audio_file = {'file': ('input.wav', open('input.wav', 'rb'))}  # Use 'file' instead of 'audio_file'

    # Creating the request URL
    url = f"{front_url}/speech?language={language}&robot_index={robot_index}"

    # Getting a response from the post request
    response = post_request(url=url, audio_file=audio_file)

    # Check if the response is successful (status code 200)
    # while True:
    if response.ok:
        try:
            # The Question Text recognized by Curio
            question = response.headers["QUESTION"]
            print("QUESTION: ", question)

            if "take me to exhibit one" in question or "exhibit one" in question:
                print("Ok taking to exhibit")
                navigation_publisher.publish_command(1)
            elif "take me to exhibit two" in question or "two" in question:
                print("Ok taking to exhibit two")
                navigation_publisher.publish_command(2)

            else:
                pass
            # The Answer Text that Curio provided
            answer = response.headers["ANSWER"]
            print("ANSWER: ", answer)

            # The Answer Text that Curio provided
            time_taken = response.headers["TIME_TAKEN"]
            print("TIME_TAKEN: ", time_taken)

            # Writing Curio's audio response into an audio file with extension .mp3
            output_file = f"output_{robot_index}.mp3"
            open(output_file, "wb").write(response.content)

            playsound.playsound('output_5.mp3', True)
            print("played")

        except KeyError as e:
            print(f"KeyError: {e}")
            print("The expected key is not present in the response headers.")
    else:
        print(f"Error: {response.status_code}")
        print(f"There was an error in the API request. {response.text}")


# Manually set the model path
model_path = "/home/jetson/wake_word_detection/hello_curio88.onnx"
inference_framework = 'onnx'

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 1280
audio = pyaudio.PyAudio()
mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

owwModel = Model(wakeword_models=[model_path], inference_framework=inference_framework)

# Initialize debounce parameters
last_detection_time = 0
debounce_interval = 1  # Time in seconds
chat_lock = threading.Lock()

# Wakeword detection logic
def detect_wakeword(navigation_publisher):
    global last_detection_time

    audio_data = np.frombuffer(mic_stream.read(CHUNK), dtype=np.int16)
    prediction = owwModel.predict(audio_data)

    for mdl in owwModel.prediction_buffer.keys():
        scores = list(owwModel.prediction_buffer[mdl])
        if scores[-1] > 0.3:  # Threshold for wakeword detection
            current_time = time.time()
            if current_time - last_detection_time > debounce_interval:
                print("Wakeword Detected!")
                last_detection_time = current_time

                with chat_lock:
                    threading.Thread(target=chat, args=(navigation_publisher,)).start()


def main(args=None):
    rclpy.init(args=args)
    navigation_publisher = NavigationPublisher()

    print("Listening for wakewords...\n")

    try:
        while rclpy.ok():
            detect_wakeword(navigation_publisher)
    except KeyboardInterrupt:
        print("Shutting down...")

    # Shutdown ROS 2
    navigation_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    
