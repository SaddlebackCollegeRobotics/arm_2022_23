import time
from roboclaw_3 import Roboclaw

#Windows comport name
# rc = Roboclaw("COM11",115200)
#Linux comport name
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80

rc.BackwardM1(address,70)	#1/4 power backward
time.sleep(7)
rc.ForwardM1(address,70)	#1/4 power forward
time.sleep(7)
rc.ForwardM1(address, 0)