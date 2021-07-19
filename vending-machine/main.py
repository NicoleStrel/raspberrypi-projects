########################################################################
# Filename    : main.py
# Description : Produces the functunality of a vending machine
########################################################################

import RPi.GPIO as GPIO
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

import time
import Keypad


# -------COMPONENT SETUP------

LED_PINS = [11, 12, 13] #R, G, B

#sensor
SENSOR_TRIGGER = 16
SENSOR_ECHO = 18
MAX_DISTANCE = 220 
timeOut = MAX_DISTANCE*60 
COIN_DISTANCE = 6.6 #cm

#keypad
ROWS = 4 
COLS = 4
CODE_1="A619"
CODE_2="D271"
keys =  [   '1','2','3','A',    
            '4','5','6','B',
            '7','8','9','C',
            '*','0','#','D'     ]
rowsPins = [32,36,38,40]
colsPins = [19,33,35,37]    

#lcd
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

#step motors
motorPins1 = (21, 23, 29, 31) 
motorPins2 = (22, 24, 26, 15)
CCWStep = (0x01,0x02,0x04,0x08) # power supply order for rotating anticlockwise 
CWStep = (0x08,0x04,0x02,0x01)  # power supply order for rotating clockwise



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

    #step motors
    for pin in motorPins1:
        GPIO.setup(pin,GPIO.OUT)
    
    for pin2 in motorPins2:
        GPIO.setup(pin2,GPIO.OUT)
    

def setColor(r_val,g_val,b_val):  
    pwmRed.ChangeDutyCycle(r_val) 
    pwmGreen.ChangeDutyCycle(g_val)   
    pwmBlue.ChangeDutyCycle(b_val)

def pulseIn(pin,level,timeOut): # obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0
    pulseTime = (time.time() - t0)*1000000
    return pulseTime

def getSonarDistance():
    GPIO.output(SENSOR_TRIGGER,GPIO.HIGH) 
    time.sleep(0.00001)     # 10us
    GPIO.output(SENSOR_TRIGGER,GPIO.LOW)
    pulseTime = pulseIn(SENSOR_ECHO,GPIO.HIGH,timeOut)
    distance = pulseTime * 340.0 / 2.0 / 10000.0     # sound speed 340m/s 
    print('distance: ', distance)
    return distance

def moveOnePeriod(direction,ms, motorPins): 
    for j in range(0,4,1):     
        for i in range(0,4,1): 
            if (direction == 1): #clockwise
                GPIO.output(motorPins[i],((CCWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
            else :             
                GPIO.output(motorPins[i],((CWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
        if(ms<3):     
            ms = 3
        time.sleep(ms*0.001)    

def moveSteps(steps, motorPins):
    for i in range(steps):
        moveOnePeriod(1, 3, motorPins)

def motorStop(motorPins):
    for i in range(0,4,1):
        GPIO.output(motorPins[i],GPIO.LOW)

def loop():
    stage_1= True #user inserts 50 cents
    stage_2= True #user enters the code of the item
    stage_3= True #motors turn to release the item
    
    count=0
    distance=0
    noCodeEntered = True
    codeEntered =''

    #set up key pad
    keypad = Keypad.Keypad(keys,rowsPins,colsPins,ROWS,COLS)
    keypad.setDebounceTime(50)   
    
    # ------------STAGE 1: ---------------
    print("Entering stage 1: Recieving coins")
    setColor(0,100,100) # red light
    
    while stage_1:
        lcd.setCursor(0,0)
        lcd.message("Please insert 50"+'\n'+"cents")
        
        distance = getSonarDistance()
        
        if distance != 0 and distance < COIN_DISTANCE:
            count+=1

        if count == 2:
            stage_1 = False
            print("Stage 1 completed")

    
    # ------------STAGE 2: ---------------
    print("Entering stage 2: Recieving code")
    setColor(100,0,0) #green light
    lcd.clear()

    while stage_2:
        if (noCodeEntered):
            lcd.setCursor(0,0)
            lcd.message("Please enter a "+'\n'+"code")
        
        key = keypad.getKey()      
        if(key != keypad.NULL):
            noCodeEntered = False
            if (key == '*'):
                codeEntered = codeEntered[:-1]
            
            else:
                codeEntered += key
            print ("codeEntered: ", codeEntered)
            
            lcd.clear()
            lcd.message("Code: " + codeEntered)
        
        if (codeEntered == CODE_1 or codeEntered == CODE_2):
            stage_2= False
            print("Stage 2 completed")
    
    # ------------STAGE 3: ---------------
    print("Entering stage 3: Dispensing")
    lcd.clear()
    lcd.setCursor(0,0)
    lcd.message("Thank you for"+'\n'+"purchasing!")

    if (codeEntered == CODE_1):
        print("Code 1 dispensing")
        moveSteps(512, motorPins1)
        motorStop(motorPins1)  
    else: 
        print("Code 2 dispensing")
        moveSteps(512, motorPins2) 
        motorStop(motorPins2)
    
    print("done!")
    destroy()

def destroy():
    lcd.clear()
    GPIO.output(LED_PINS, GPIO.LOW)
    GPIO.cleanup()

if __name__ == '__main__': 
    print ('Vending Machine is starting ... \n')
    setup()
    try:
        loop()
    except KeyboardInterrupt: # ctrl-c
        destroy()
