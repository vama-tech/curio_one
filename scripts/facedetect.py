import numpy as np
import cv2
import mediapipe as mp
import time
import pyttsx3
import threading

engine = pyttsx3.init()

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=5, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=2, circle_radius=1)

cap = cv2.VideoCapture(0)
delay_set = 1
person_detected = False
person_start_time = None
warning_spoken = False

# Function to handle speech in a separate thread
def say_warning():
    global warning_spoken
    engine.say("Please Move Aside")
    engine.runAndWait()
    warning_spoken = False

while cap.isOpened():
    success, image = cap.read()

    start = time.time()

    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)  # flipped for selfie view
    image.flags.writeable = False
    results = face_mesh.process(image)
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    img_h, img_w, img_c = image.shape

    if results.multi_face_landmarks:
        if person_start_time is None:
            person_start_time = time.time()
        
        elif not warning_spoken and time.time() - person_start_time > delay_set:
            warning_spoken = True
            threading.Thread(target=say_warning).start()  # Start the warning thread
        print("Person Detected")

        end = time.time()
        totalTime = end - start
        fps = 1 / totalTime

        cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
    else:
        print("No Person Detected")
        warning_spoken = False  # Reset the flag if no person is detected
        person_start_time = None

    cv2.imshow('Head Pose Detection', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
