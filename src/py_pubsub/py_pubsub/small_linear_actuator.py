import RPi.GPIO as GPIO
import time

class Alfreds_Finger:
    PUSH = 11
    PULL = 12
    FEED_TIME = 2.5


    def __init__(self):
        """
        Initialize the Linear Actuator class and set up GPIO pins for finger control 😳😳😳.
        """
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PUSH, GPIO.OUT)
        GPIO.setup(self.PULL, GPIO.OUT)


    def push(self):
        """
        Push the finger out. 😳👉
        """
        GPIO.output(Alfreds_Finger.PUSH, True)
        time.sleep(Alfreds_Finger.FEED_TIME)
        GPIO.output(Alfreds_Finger.PULL, False)


    def pull(self):
        """
        Pull the finger in. 👈😳
        """
        GPIO.output(Alfreds_Finger.PULL, True)
        time.sleep(Alfreds_Finger.FEED_TIME)
        GPIO.output(Alfreds_Finger.PUSH, False)

    
    def cleanup(self):
        """
        Clean up GPIO pins.
        """
        GPIO.cleanup()