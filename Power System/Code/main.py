from machine import Pin, ADC,I2C,UART
from time import sleep
from picoslave import *
Vdivide = ADC(Pin(28))
Ishunt = ADC(Pin(27)) #Define Pins & setup realay 1 to enable 5v
LED = Pin("LED", Pin.OUT)
R1 = Pin(15,Pin.OUT)
R2 = Pin(14,Pin.OUT)
R3 = Pin(13,Pin.OUT)
R4 = Pin(12,Pin.OUT)
R1.on()
Sig1 = Pin(4,Pin.OUT)
Sig2 = Pin(5,Pin.OUT)
# i2c = i2c_slave(0,sda=4,scl=5,slaveAddress=0x41)
#uart = UART(1, baudrate=9600, tx=Pin(4),rx=Pin(5))
#uart.init(bits=8, parity=None,stop=1)
while True:
    Ishunt_Value = Ishunt.read_u16()/21845 #Grab current reading
    Vdivide_value = Vdivide.read_u16()/21845  #Grab voltage reading
    print("battery Volts Out:") #print to console for debug
    print(Vdivide_value*40)
    Vval=Vdivide_value*40 #Calculate real voltage
    print("Charge Current in:")
    print(Ishunt_Value/(1.1/6))
    Ishunt_Value=Ishunt_Value/(1.1/6) #calculate real current
    if Ishunt_Value > 1: # Are we charging?
        R1.off()
        R2.off()  #Enter Standby Mode
        R3.off()
        R4.off()
        Sig1.on() # Tell control sys we are charging
        LED.on() #Indicate Charging
    else:
        R1.on()
        R2.on()
        R3.on() # Exit Standby Mode
        R4.on()
        LED.off() 
        Sig1.off()
    if Vdivide_value < 15:
        Sig2.on() # Tell control sys battery is low
    else:
        Sig2.off() # Battery no longer low
            