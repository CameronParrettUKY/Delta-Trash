# This is only a test code to test the connection between the vision system and the control system.
# This code has been integrated into the APP_Control_Speed file.
from machine import Pin, PWM, UART
import utime

# Global speed variable for automatic control
speed = 0.1  # Start at 9% speed as per your setting

# Initialize UART for automatic control from vision system
vision_system = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

# Define motor control pins
# Lift motor control pins
R_EN = Pin(2, Pin.OUT)
L_EN = Pin(3, Pin.OUT)
R_PWM = PWM(Pin(13))
L_PWM = PWM(Pin(14))
# Right motor control pins
R_EN_2 = Pin(27, Pin.OUT)
L_EN_2 = Pin(26, Pin.OUT)
R_PWM_2 = PWM(Pin(21))
L_PWM_2 = PWM(Pin(20))

# Initialize motor control pins
R_EN.high()
L_EN.high()
R_PWM.freq(9600)
L_PWM.freq(9600)

R_EN_2.high()
L_EN_2.high()
R_PWM_2.freq(9600)
L_PWM_2.freq(9600)

# Function to control both motors
def motor_control(speed, direction):
    duty_cycle = int(speed * 65535)
    # Adjust the motor control logic as per the new direction commands
    if direction == "forward":
        R_PWM.duty_u16(duty_cycle)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(duty_cycle)
    elif direction == "backward":
        R_PWM.duty_u16(0)
        L_PWM.duty_u16(duty_cycle)
        R_PWM_2.duty_u16(duty_cycle)
        L_PWM_2.duty_u16(0)
    elif direction == "stop":  # stop or any other command
        R_PWM.duty_u16(0)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(0)

# Main loop for automatic control based on vision system input
try:
    while True:
        if vision_system.any():
            command = vision_system.read().decode('utf-8').strip()
            # Map vision system commands to motor control
            print("Received command:", command)
            if command == 'F':
                motor_control(speed, "forward")
            elif command == 'B':
                motor_control(speed, "backward")
            elif command == 'S':
                motor_control(speed, "stop")

        utime.sleep(0.1)  # Delay to prevent CPU overload

except KeyboardInterrupt:
    # Cleanup
    R_PWM.deinit()
    L_PWM.deinit()
    R_EN.low()
    L_EN.low()
    R_PWM_2.deinit()
    L_PWM_2.deinit()
    R_EN_2.low()
    L_EN_2.low()
