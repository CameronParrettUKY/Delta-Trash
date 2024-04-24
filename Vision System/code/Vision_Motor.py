import RPi.GPIO as GPIO
import time
import serial
import cv2
import numpy as np
import threading

serial_port = '/dev/ttyS0'  # Use '/dev/ttyS0' for Raspberry Pi 3/4, '/dev/ttyAMA0' for Raspberry Pi 1/2
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate)
print("Serial port opened")

# Suppress GPIO warnings
GPIO.setwarnings(False)

# Set up GPIO for Ultrasonic sensors, LED
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER_FRONT = 23
GPIO_ECHO_FRONT = 24
GPIO_TRIGGER_LEFT = 17
GPIO_ECHO_LEFT = 27
GPIO_TRIGGER_RIGHT = 22
GPIO_ECHO_RIGHT = 25
GPIO_TRIGGER_BACK = 5
GPIO_ECHO_BACK = 6
GPIO_LED = 18  # GPIO pin connected to the LED
GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_BACK, GPIO.OUT)
GPIO.setup(GPIO_ECHO_BACK, GPIO.IN)
GPIO.setup(GPIO_LED, GPIO.OUT)

# Function to stop motors
def stop_motors():
    # No GPIO operations since motors are not connected
    pass

# Functions for ultrasonic sensors
def measure_distance(trigger_pin, echo_pin):
    """
    This function measures the distance detected by the ultrasonic sensor.
    Returns the distance in centimeters.
    """
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

# Function to initialize and operate Lidar
def lidar_scan():
    # No Lidar functionality in this version
    pass

# Function to blink the LED
def blink_led(times, duration):
    """
    Blinks the LED a specified number of times with a specified duration.
    """
    for _ in range(times):
        GPIO.output(GPIO_LED, True)
        time.sleep(duration)
        GPIO.output(GPIO_LED, False)
        time.sleep(duration)

# Define the height of the obstacle (in meters)
obstacle_height = 1.7  # Example: height of a person

# Define the size threshold for a tennis ball (in pixels)
tennis_ball_size = 40  # Example: size of a tennis ball

# Function to detect obstacles, draw squares, and add distance information
def detect_obstacles(frame, focal_length):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection using Canny
    edges = cv2.Canny(blurred, 30, 150)
    
    # Find contours of the detected edges
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw squares and add distance information
    obstacle_frame = frame.copy()
    for contour in contours:
        # Find the bounding rectangle of the contour
        x, y, w, h = cv2.boundingRect(contour)
        
        # Check if the object size exceeds the size threshold for a tennis ball
        if max(w, h) < tennis_ball_size:
            # Skip small objects (assuming tennis ball size as reference)
            continue
        
        # Draw a thin rectangle around the contour with a specific color
        rectangle_color = (0, 255, 0)  # Green color for rectangles
        cv2.rectangle(obstacle_frame, (x, y), (x + w, y + h), rectangle_color, 1)
        
        # Estimate the distance using simple geometry (distance = focal_length * real_height / apparent_height)
        apparent_height = h
        distance = (obstacle_height * focal_length) / apparent_height
        
        # Add distance information to the rectangle with a different color
        text_color = (0, 0, 255)  # Red color for distance text
        cv2.putText(obstacle_frame, f"{distance:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
    
    return obstacle_frame

# Function to capture video and perform obstacle detection
def video_thread():
    # Capture video from the camera
    cap = cv2.VideoCapture(0)

    # Set the focal length (in pixels)
    # This can be calculated using the camera's focal length and sensor size, or estimated experimentally
    focal_length = 1000  # Example value

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            break
        
        # Split the frame into two screens
        obstacle_frame = detect_obstacles(frame, focal_length)
        original_frame = frame

        # Display the frames in separate windows
        cv2.imshow('Obstacles', obstacle_frame)
        cv2.imshow('Original', original_frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

    # Release the camera and close the windows
    cap.release()
    cv2.destroyAllWindows()

# Start the video capture thread
video_thread = threading.Thread(target=video_thread)
video_thread.start()

try:
    while True:
        # Measure distances from the front, left, right, and back sensors
        dist_front = measure_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT)
        dist_left = measure_distance(GPIO_TRIGGER_LEFT, GPIO_ECHO_LEFT)
        dist_right = measure_distance(GPIO_TRIGGER_RIGHT, GPIO_ECHO_RIGHT)
        dist_back = measure_distance(GPIO_TRIGGER_BACK, GPIO_ECHO_BACK)

        print("Front Distance: %.1f cm" % dist_front)
        print("Left Distance: %.1f cm" % dist_left)
        print("Right Distance: %.1f cm" % dist_right)
        print("Back Distance: %.1f cm" % dist_back)

        # Add your obstacle avoidance logic here using the distance readings
        # For example:
        if dist_front < 20 or dist_left < 20 or dist_right < 20 or dist_back < 20:
            # Send stop command to Pico
            ser.write(b'S\n')
            # Stop motors
            stop_motors()
            # Blink the LED
            blink_led(10, 0.1)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()  # Clean up GPIO resources
    # Cleanup operations if necessary
    video_thread.join()  # Wait for the video thread to finish
