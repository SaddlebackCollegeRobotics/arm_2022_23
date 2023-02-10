import sys
import time

# PATH_TO_ROBOCLAW = "../../lib/roboclaw_python"
# sys.path.append(PATH_TO_ROBOCLAW)

from .roboclaw_3 import Roboclaw

# Windows comport name - "COM8"
# Linux comport name - "/dev/ttyACM1"
# OSX comport name - "/dev/tty.usbmodem1301"
COMPORT_NAME = "/dev/ttyACM0"

SPEED = 850 #600
ACCELERATION = 700 #200



class Encoder:

    def __init__(self, encoder_extend: int, encoder_retract: int, angle_extend: float, angle_retract: float):
        self.encoder_extend = encoder_extend
        self.encoder_retract = encoder_retract
        self.angle_extend = angle_extend
        self.angle_retract = angle_retract

    # Convert angle to encoder value taking into account min/max encoder values and angle values.
    def angle_to_enc(self, angle : float):
        slope = (self.encoder_extend - self.encoder_retract) / (self.angle_extend - self.angle_retract)
        
        return int(slope * (angle - self.angle_retract) + self.encoder_retract)


# Motor controller instance.
class Motor_Controller:
    
    def __init__(self, rc: Roboclaw, address: int, m1: Encoder, m2: Encoder):

        self.rc = rc
        self.address = address
        self.m1 = m1
        self.m2 = m2 

        self.rc.Open()

    # Print speed and encoder values of both motors to terminal
    def print_encoders(self):
        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected")
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")


# Set position of arm using PID function.
def set_arm_position(shoulderJointAngle : float, elbowJointAngle : float, baseRotationAngle):
    
    encoder_val_m1 = mcp.m1.angle_to_enc(elbowJointAngle)
    encoder_val_m2 = mcp.m2.angle_to_enc(shoulderJointAngle)

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        1  # 0 for buffer, 1 for instant write
    )

    print(shoulderJointAngle, " ", elbowJointAngle, " ", baseRotationAngle)

# Create instance of a motor controller.
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
# rm -rf build install log && colcon build && source ./install/setup.bash && ros2 run py_pubsub listener