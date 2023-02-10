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
        if angle > self.angle_retract: angle = self.angle_retract
        if angle < self.angle_extend: angle = self.angle_extend

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
        print(f"M1: {enc[1]}" if enc[0] == 1 else "disconnected")
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc[1]}" if enc[0] == 1 else "disconnected")



def main():
    mcp = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),
        address = 0x80,  
        m1 = Encoder(  # forearm
            encoder_extend = 20,
            encoder_retract = 1960,
            angle_extend = 90 - 72.51,  # 17.49
            angle_retract = 90 - 35.68  # 53.32
        ),
        m2 = Encoder(  # bicep
            encoder_extend = 2337,
            encoder_retract = 288,
            angle_extend = 90 - 66.65,  # 23.35
            angle_retract = 90 - 15.47  # 74.53
        )
    )
    
    encoder_val_m1 = mcp.m1.angle_to_enc(float(sys.argv[1]) if len(sys.argv) > 1 else 45)
    encoder_val_m2 = mcp.m2.angle_to_enc(float(sys.argv[2]) if len(sys.argv) > 2 else 45)

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        0  # 0 for buffer, 1 for instant write
    )

if __name__ == '__main__':
    main()
