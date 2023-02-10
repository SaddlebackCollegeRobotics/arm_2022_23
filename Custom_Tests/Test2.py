import time
from roboclaw_3 import Roboclaw

# Functions

def printEncoder():

    enc1 = rc.ReadEncM2(address)

    if(enc1[0]==1):
        print (enc1[1])
        print (format(enc1[2],'02x'))
    else:
        print ("failed")

# Main

#Windows comport name
# rc = Roboclaw("COM8",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM1",115200)

rc.Open()
address = 0x80

printEncoder()
# rc.SpeedAccelDeccelPositionM2(address, 20, 50, 20, 2000, 1)
# rc.SpeedDistanceM2(address, 30, -1000, 1)
# rc.SpeedAccelM2(address, 10, -30)

# time.sleep(10)
printEncoder()