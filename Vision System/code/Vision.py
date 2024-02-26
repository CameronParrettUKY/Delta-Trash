import cv2
import numpy as np
import threading
import RPi.GPIO as GPIO
import time
import serial
from picamera import PiCamera

# Set up GPIO for Ultrasonic sensors, LED, and Motors
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER_FRONT = 23
GPIO_ECHO_FRONT = 24
GPIO_TRIGGER_LEFT = 17
GPIO_ECHO_LEFT = 27
GPIO_TRIGGER_RIGHT = 22
GPIO_ECHO_RIGHT = 25
GPIO_LED = 18

# Set up GPIO direction (IN for ECHO, OUT for TRIGGER and LED)
GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)
GPIO.setup(GPIO_LED, GPIO.OUT)

# Set up Pi Camera
camera = PiCamera()
camera.resolution = (1024, 576)
camera.framerate = 24

# Global variables
blink_count = 10

# Serial port for communication with Raspberry Pi Pico
serial_port = serial.Serial('/dev/ttyS0', 9600)  # Adjust port and baud rate as needed

# Motor control functions
def forward():
    serial_port.write(b'F')

def backward():
    serial_port.write(b'B')

def stop_motors():
    serial_port.write(b'S')

# Functions for ultrasonic sensors and LED
def measure_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, False)
    time.sleep(0.1)

    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = start_time

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2

    return distance

def blink_led(times, duration):
    for _ in range(times):
        GPIO.output(GPIO_LED, True)
        time.sleep(duration)
        GPIO.output(GPIO_LED, False)
        time.sleep(duration)

# Load pre-trained Haar cascade classifiers for face and full-body detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
fullbody_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

# Initialize background subtractor for detecting movements
bg_subtractor = cv2.createBackgroundSubtractorMOG2()

# Define colors for different objects
COLORS = {
    'face': (255, 0, 0),    # Blue
    'body': (0, 255, 0),    # Green
    'movement': (0, 0, 255)  # Red
}

# Function to detect and draw faces
def detect_faces(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), COLORS['face'], 2)
        cv2.putText(frame, f"Distance: {measure_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT):.1f} cm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS['face'], 2)

# Function to detect and draw full bodies
def detect_bodies(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bodies = fullbody_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    for (x, y, w, h) in bodies:
        cv2.rectangle(frame, (x, y), (x+w, y+h), COLORS['body'], 2)
        cv2.putText(frame, f"Distance: {measure_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT):.1f} cm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS['body'], 2)

# Function to detect and draw movements
def detect_movements(frame):
    fg_mask = bg_subtractor.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w > 50 and h > 50:  # Ignore small movements
            cv2.rectangle(frame, (x, y), (x+w, y+h), COLORS['movement'], 2)
            cv2.putText(frame, f"Distance: {measure_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT):.1f} cm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS['movement'], 2)

# Function to start live video streaming
def start_video_stream():
    cv2.namedWindow("Video Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Video Stream", 1024, 576)
    while True:
        frame = np.empty((576, 1024, 3), dtype=np.uint8)
        camera.capture(frame, 'bgr', use_video_port=True)
        
        # Multithreading for faster processing
        threads = []
        for func in [detect_faces, detect_bodies, detect_movements]:
            thread = threading.Thread(target=func, args=(frame,))
            threads.append(thread)
            thread.start()
        
        for thread in threads:
            thread.join()
        
        cv2.imshow('Video Stream', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()

try:
    # Start sensor thread for live video streaming
    video_thread = threading.Thread(target=start_video_stream)
    video_thread.start()

    while True:
        # Measure distances from the front, left, and right sensors
        dist_front = measure_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT)
        dist_left = measure_distance(GPIO_TRIGGER_LEFT, GPIO_ECHO_LEFT)
        dist_right = measure_distance(GPIO_TRIGGER_RIGHT, GPIO_ECHO_RIGHT)

        print("Front Distance: %.1f cm" % dist_front)
        print("Left Distance: %.1f cm" % dist_left)
        print("Right Distance: %.1f cm" % dist_right)

        # Add obstacle avoidance logic using the distance readings
        if dist_front < 30 or dist_left < 30 or dist_right < 30:
            blink_led(blink_count, 0.1)  # Blink the LED
            
            backward()  # Execute avoidance maneuver
            time.sleep(1)
            stop_motors()
            # Add motor control logic for turning right
        else:
            forward()  # Move forward
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Measurement stopped by User")
    serial_port.close()  # Close serial port
    GPIO.cleanup()
    camera.close()