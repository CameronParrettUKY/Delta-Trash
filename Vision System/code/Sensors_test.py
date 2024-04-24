import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the ultrasonic sensors and LED
GPIO_TRIGGER_FRONT = 23
GPIO_ECHO_FRONT = 24
GPIO_TRIGGER_LEFT = 17
GPIO_ECHO_LEFT = 27
GPIO_TRIGGER_RIGHT = 22
GPIO_ECHO_RIGHT = 25
GPIO_TRIGGER_BACK = 5
GPIO_ECHO_BACK = 6
GPIO_LED = 18

# Set GPIO direction (IN for ECHO, OUT for TRIGGER and LED)
GPIO.setup(GPIO_TRIGGER_FRONT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_FRONT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_BACK, GPIO.OUT)
GPIO.setup(GPIO_ECHO_BACK, GPIO.IN)
GPIO.setup(GPIO_LED, GPIO.OUT)

def measure_distance(trigger_pin, echo_pin):
    """
    This function measures the distance detected by the ultrasonic sensor.
    Returns the distance in centimeters.
    """
    # Ensure trigger is low
    GPIO.output(trigger_pin, False)
    time.sleep(0.00001)  # Short delay
    
    # Send a 10us pulse to trigger
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)  # Keep trigger high for 10us
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = start_time

    # Save start time
    while GPIO.input(echo_pin) == 0 and time.time() - start_time < 0.1:  # Timeout after 0.1 seconds
        start_time = time.time()

    # Save arrival time
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    # Time difference between start and arrival
    elapsed_time = stop_time - start_time
    # Multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (elapsed_time * 34300) / 2

    return distance

def blink_led(times, duration):
    """
    Blinks the LED a specified number of times with a specified duration.
    """
    for _ in range(times):
        GPIO.output(GPIO_LED, True)
        time.sleep(duration)
        GPIO.output(GPIO_LED, False)
        time.sleep(duration)

try:
    while True:
        dist_front = measure_distance(GPIO_TRIGGER_FRONT, GPIO_ECHO_FRONT)
        dist_left = measure_distance(GPIO_TRIGGER_LEFT, GPIO_ECHO_LEFT)
        dist_right = measure_distance(GPIO_TRIGGER_RIGHT, GPIO_ECHO_RIGHT)
        dist_back = measure_distance(GPIO_TRIGGER_BACK, GPIO_ECHO_BACK)
        
        print("Front Distance = %.1f cm, Left Distance = %.1f cm, Right Distance = %.1f cm, Back Distance = %.1f cm" % (dist_front, dist_left, dist_right, dist_back))
        
        if dist_front < 30 or dist_left < 30 or dist_right < 30 or dist_back < 30:  # Adjust this value as needed
            blink_led(5, 0.1)  # Blink 5 times, each blink lasting 0.1 seconds
        else:
            GPIO.output(GPIO_LED, False)  # Turn off LED if not in range
        
        time.sleep(0.1)  # Adjust sleep duration here

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()  # Clean up GPIO
