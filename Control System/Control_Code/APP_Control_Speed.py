# This code applies to devices made by the Trash Boss 2024 Spring team.
# Use the Raspberry Pi Pico W on the edge of the PCB for control. 
# The existing main.py file for the Raspberry Pi Pico W already has code saved for use when connecting directly to a power supply.

from machine import UART, Pin, PWM
import utime

# Global speed variable
speed = 0.3  # Start at 30% speed
armspeed = 0.8
direction = "stop"

# Initialize Bluetooth and Vision System UART
bluetooth = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
VSuart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) 

R_EN_Arm = Pin(10, Pin.OUT)
L_EN_Arm = Pin(11, Pin.OUT)
R_PWM_Arm = PWM(Pin(8))
L_PWM_Arm = PWM(Pin(9))

# Define lift motor control pins
R_EN = Pin(27, Pin.OUT)
L_EN = Pin(28, Pin.OUT)
R_PWM = PWM(Pin(16)) #616
L_PWM = PWM(Pin(17)) #717
# Define right motor control pins
R_EN_2 = Pin(22, Pin.OUT)
L_EN_2 = Pin(26, Pin.OUT)
R_PWM_2 = PWM(Pin(18))
L_PWM_2 = PWM(Pin(21))

R_EN_Brk = Pin(14, Pin.OUT)
L_EN_Brk = Pin(15, Pin.OUT)
R_PWM_Brk = PWM(Pin(12))
L_PWM_Brk = PWM(Pin(13))

# Initialize motor control pins
R_EN.high()
L_EN.high()
R_PWM.freq(1000)
L_PWM.freq(1000)

R_EN_2.high()
L_EN_2.high()
R_PWM_2.freq(1000)
L_PWM_2.freq(1000)
    
R_EN_Arm.high()
L_EN_Arm.high()
R_PWM_Arm.freq(1000)
L_PWM_Arm.freq(1000)

R_EN_Brk.high()
L_EN_Brk.high()
R_PWM_Brk.freq(1000)
L_PWM_Brk.freq(1000)

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
        L_PWM.duty_u16(duty_cycle)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(duty_cycle)
    elif direction == "right":
        R_PWM.duty_u16(duty_cycle)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(duty_cycle)
        L_PWM_2.duty_u16(0)
    elif direction == "stop":
        R_PWM.duty_u16(0)
        L_PWM.duty_u16(0)
        R_PWM_2.duty_u16(0)
        L_PWM_2.duty_u16(0)
        R_PWM_Arm.duty_u16(0)
        L_PWM_Arm.duty_u16(0)
        R_PWM_Brk.duty_u16(0)
        L_PWM_Brk.duty_u16(0)
    elif direction == "arm up":
        R_PWM_Arm.duty_u16(0)
        L_PWM_Arm.duty_u16(duty_cycle)
    elif direction == "arm down":
        R_PWM_Arm.duty_u16(duty_cycle)
        L_PWM_Arm.duty_u16(0)
    elif direction == "brake on":
        R_PWM_Brk.duty_u16(duty_cycle)
        L_PWM_Brk.duty_u16(0)
    elif direction == "brake off":
        R_PWM_Brk.duty_u16(0)
        L_PWM_Brk.duty_u16(duty_cycle)
  

# Main loop
try:
    while True:
        
        while VSuart.any():
            commandVS = VSuart.readline().decode("utf-8").strip()
            print("Received command:", commandVS) 

            if commandVS == 'S':
                motor_control(0, "stop")
                continue
           
        if bluetooth.any():
            command = bluetooth.read(1).decode('utf-8')
            print("Moving command:", command)
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
        
            elif command == '1':
                direction = "arm up"
                motor_control(armspeed, direction)

            elif command == '3':
                direction = "arm down"
                motor_control(armspeed, direction)

            elif command == 'a':
                direction = "brake on"
                motor_control(armspeed, direction)
            elif command == 'b':
                direction = "brake off"
                motor_control(armspeed, direction)

            elif command == '7':  # Increase speed
                speed = min(1, speed + 0.1)
                if direction != "stop":  # Only change speed if not stopped
                    motor_control(speed, direction)
            elif command == '9':  # Decrease speed
                speed = max(0, speed - 0.1)
                if direction != "stop":  # Only change speed if not stopped
                    motor_control(speed, direction)
        utime.sleep(0.5)
        
except KeyboardInterrupt:
    R_PWM.deinit()
    L_PWM.deinit()
    R_EN.low()
    L_EN.low()
    R_PWM_2.deinit()
    L_PWM_2.deinit()
    R_EN_2.low()
    L_EN_2.low()
    R_EN_Arm.low()
    L_EN_Arm.low()
    R_PWM_Arm.deinit()
    L_PWM_Arm.deinit()
    R_EN_Brk.low()
    L_EN_Brk.low()
    R_PWM_Brk.deinit()
    L_PWM_Brk.deinit()

