from machine import UART, Pin, PWM
import utime

# Global speed variable
speed = 0.3  # Start at 30% speed
direction = "stop"

# Initialize Bluetooth UART
bluetooth = UART(0, baudrate=9600, tx=Pin(16), rx=Pin(17))

# Define lift motor control pins
R_EN = Pin(2, Pin.OUT)
L_EN = Pin(3, Pin.OUT)
R_PWM = PWM(Pin(13))
L_PWM = PWM(Pin(14))
# Define right motor control pins
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

# Function to control motor
def motor_control(speed, direction):
    duty_cycle = int(speed * 65535)
    
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
    elif direction == "lift":
        R_PWM.duty_u16(0)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(duty_cycle)
    elif direction == "right":
        R_PWM.duty_u16(duty_cycle)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(0)
    else:
        R_PWM.duty_u16(0)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(0)

# Main loop
try:
    while True:
        if bluetooth.any():
            command = bluetooth.read(1).decode('utf-8')
            if command == '2':
                direction = "forward"
                motor_control(speed, direction)
            elif command == '8':
                direction = "backward"
                motor_control(speed, direction)
            elif command == '5':
                direction = "stop"
                motor_control(0, direction)
                
            elif command == '4':
                direction = "lift"
                motor_control(speed, direction)
            elif command == '6':
                direction = "right"
                motor_control(speed, direction)
                
            elif command == '7':  # Increase speed
                speed = min(1, speed + 0.1)
                if direction != "stop":  # Only change speed if not stopped
                    motor_control(speed, direction)
            elif command == '9':  # Decrease speed
                speed = max(0, speed - 0.1)
                if direction != "stop":  # Only change speed if not stopped
                    motor_control(speed, direction)

except KeyboardInterrupt:
    R_PWM.deinit()
    L_PWM.deinit()
    R_EN.low()
    L_EN.low()
    R_PWM_2.deinit()
    L_PWM_2.deinit()
    R_EN_2.low()
    L_EN_2.low()
