# Author: Jasper Doan - @wluxie, Cameron Rosenthal - @Supernova1114

import RPi.GPIO as GPIO

class Alfreds_Finger:

    PUSH = 11
    PULL = 13
    FEED_TIME = 2.5

    LAST_STATE = 0

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

        if self.LAST_STATE == 1:
            return
        
        self.LAST_STATE = 1
        GPIO.output(Alfreds_Finger.PUSH, True)


    def pull(self):
        """
        Pull the finger in. 👈😳
        """

        if self.LAST_STATE == -1:
            return
        
        self.LAST_STATE = -1
        GPIO.output(Alfreds_Finger.PULL, True)


    def stop(self):
        """
        Stop the finger movement. 😳😳😳
        """

        self.LAST_STATE = 0

        GPIO.output(Alfreds_Finger.PUSH, False)
        GPIO.output(Alfreds_Finger.PULL, False)
    

    def cleanup(self):
        """
        Clean up GPIO pins.
        """
        GPIO.cleanup()
