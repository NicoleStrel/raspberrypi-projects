import RPi.GPIO as GPIO    # Import Raspberry Pi GPIO library
from time import sleep     # Import the sleep function from the time module

GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BOARD)   # Use physical pin numbering
GPIO.setup(11, GPIO.OUT, initial=GPIO.LOW)   # Set pin 12 to be an output pin and set initial value to low (off)

print ("running")
while True: # Run forever
    GPIO.output(11, GPIO.HIGH) # Turn on
    print ("LED ON")
    sleep(1)                  # Sleep for 1 second
    GPIO.output(11, GPIO.LOW)  # Turn off
    print ("LED OFF")
    sleep(1)                  # Sleep for 1 second

