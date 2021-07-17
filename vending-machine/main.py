########################################################################
# Filename    : main.py
# Description : Produces the functunality of a vending machine
########################################################################

import RPi.GPIO as GPIO
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

from time import sleep


# ---COMPONENT SETUP ---

LED_PINS = [11, 12, 13] #R, G, B


SENSOR_TRIGGER = 16
SENSOR_ECHO = 18


# ------ LCD SETUP ------
PCF8574_address = 0x27 
PCF8574A_address = 0x3F  

try:
    mcp = PCF8574_GPIO(PCF8574_address)
except:
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print ('I2C Address Error !')
        exit(1)
# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)


# ---- Logic Variables -----
CODE_1="A619"
CODE_2="D271"

def setup():
    global pwmRed,pwmGreen,pwmBlue

    GPIO.setmode(GPIO.BOARD)      
    
    #led
    GPIO.setup(LED_PINS, GPIO.OUT) 
    GPIO.output(LED_PINS, GPIO.HIGH) 
    pwmRed = GPIO.PWM(LED_PINS[0], 2000)# 2kHz
    pwmGreen = GPIO.PWM(LED_PINS[1], 2000)  
    pwmBlue = GPIO.PWM(LED_PINS[2], 2000)   
    pwmRed.start(0)
    pwmGreen.start(0)
    pwmBlue.start(0)

    #sensor 
    
    GPIO.setup(SENSOR_TRIGGER, GPIO.OUT)
    GPIO.setup(SENSOR_ECHO, GPIO.IN)
    
    
    #lcd
    mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2)     # set number of LCD lines and columns
    

def setColor(r_val,g_val,b_val):  
    pwmRed.ChangeDutyCycle(r_val) 
    pwmGreen.ChangeDutyCycle(g_val)   
    pwmBlue.ChangeDutyCycle(b_val)


def checkCoins(count):
    long pingTime;
    float distance;
    GPIO.output(SENSOR_TRIGGER,GPIO.HIGH)
    if distance == 100:
        count+=1


def loop():
    stage_1= True #user inserts 50 cents
    stage_2= True #user enters the code of the item
    stage_3= True #motors turn to release the item
    
    count=0
    
    # ------------STAGE 1: ---------------
    setColor(0,100,100) # red light
    
    while stage_1:
        #print ("looping")
        lcd.setCursor(0,0)
        lcd.message("Please insert 50"+'\n'+"cents")
        
        checkCoins(count)
        if count == 2:
            stage_1 = False
        #sleep(1)
    
    # STAGE 2:
    setColor(6,100,55) #green light

def destroy():
    lcd.clear()
    GPIO.cleanup()

if __name__ == '__main__': 
    print ('Vending Machine is starting ... \n')
    setup()
    try:
        loop()
    except KeyboardInterrupt: # ctrl-c
        destroy()
