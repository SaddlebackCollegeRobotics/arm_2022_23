import sys
import time

PATH_TO_ROBOCLAW = "../../lib/roboclaw_python"
sys.path.append(PATH_TO_ROBOCLAW)

from roboclaw_3 import Roboclaw

# Windows comport name - "COM8"
# Linux comport name - "/dev/ttyACM1"
# OSX comport name - "/dev/tty.usbmodem1301"
COMPORT_NAME = "/dev/ttyACM0"

SPEED = 600
ACCELERATION = 200



class Encoder:
    def __init__(self, encoder_extend: int, encoder_retract: int, angle_extend: float, angle_retract: float):
        self.encoder_extend = encoder_extend
        self.encoder_retract = encoder_retract
        self.angle_extend = angle_extend
        self.angle_retract = angle_retract

    def angle_to_enc(self, angle):
        #if angle > self.angle_retract: angle = self.angle_retract
        #if angle < self.angle_extend: angle = self.angle_extend

        slope = (self.encoder_extend - self.encoder_retract) / (self.angle_extend - self.angle_retract)
        return int(slope * (angle - self.angle_retract) + self.encoder_retract)

class Motor_Controller:
    def __init__(self, rc: Roboclaw, address: int, m1: Encoder, m2: Encoder):
        self.rc = rc
        self.address = address
        self.m1 = m1
        self.m2 = m2 

        self.rc.Open()

    def print_encoders(self):
        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected", end=' ')
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")



def main():
    mcp = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),
        address = 0x80,  
        m1 = Encoder(  # elbow joint
            encoder_extend = 20, #20
            encoder_retract = 1930, # 1930

            angle_extend = 75,
            angle_retract = 140
        ),
        m2 = Encoder(  # shoulder joint
            encoder_extend = 30, #30
            encoder_retract = 2640, #2640
            
            angle_extend = 5,
            angle_retract = 75
        )
    )

    encoder_val_m1 = mcp.m1.angle_to_enc(float(sys.argv[1]) if len(sys.argv) > 1 else 45)
    encoder_val_m2 = mcp.m2.angle_to_enc(float(sys.argv[2]) if len(sys.argv) > 2 else 45)

    # encoder_val_m1 = int(sys.argv[1]) if len(sys.argv) > 1 else 45
    # encoder_val_m2 = int(sys.argv[2]) if len(sys.argv) > 2 else 45

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        1  # 0 for buffer, 1 for instant write
    )

    while (1 == 1 == 1 and 1 == 1 == 1):
        time.sleep(.1)
        mcp.print_encoders()


if __name__ == '__main__':
    main()
