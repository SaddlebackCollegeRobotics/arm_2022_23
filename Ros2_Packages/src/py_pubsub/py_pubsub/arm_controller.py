import math
import sys

# PATH_TO_ROBOCLAW = "../../lib/roboclaw_python"
# sys.path.append(PATH_TO_ROBOCLAW)

from roboclaw_3 import Roboclaw
from collections import namedtuple

# Windows comport name - "COM8"
# Linux comport name - "/dev/ttyACM1"
# OSX comport name - "/dev/tty.usbmodem1301"
COMPORT_NAME = "/dev/ttyACM0"

SPEED = 850 #600
ACCELERATION = 700 #200

# linear actuator position tuple
# d (inches) - length from joint of arm to linear actuator
# h (inches) - vertical distance between linear actuator base and arm hinge
LINEAR_ACTUATOR_POS = namedtuple('actuator_pos', 'd h l')

class Encoder:
    def __init__(self, encoder_extend: int, encoder_retract: int, angle_extend: float, angle_retract: float, length_extend: float, length_retract: float, actuator_pos: LINEAR_ACTUATOR_POS):
        self.encoder_extend = encoder_extend
        self.encoder_retract = encoder_retract
        self.angle_extend = angle_extend
        self.angle_retract = angle_retract
        self.length_extend = length_extend
        self.length_retract = length_retract
        self.actuator_pos = actuator_pos

    #NOTE use encoder.angle_to_enc for now, angle_to_length not currently working
    # Convert angle linear actuator lengths taking into account min/max angles and length values.
    def angle_to_length(self, angle : float) -> float:
        d, h, l = self.actuator_pos 
        actuator_length = math.sqrt( math.pow(l + d*math.cos(angle) , 2) + math.pow(h - d*math.sin(angle) , 2) )
        return actuator_length

    #NOTE correctly maps angle and encoder values at limits, but loses accuracy the closer it gets to center
    # Convert angle to encoder value taking into account min/max encoder values and angle values.
    def angle_to_enc(self, angle : float) -> int:
        slope = (self.encoder_extend - self.encoder_retract) / (self.angle_extend - self.angle_retract)
        
        return int(slope * (angle - self.angle_retract) + self.encoder_retract)

    #NOTE correctly maps between length and encoder values
    # Convert linear actuator lengths to encoder value taking into account min/max lengths and angle values.
    def length_to_enc(self, length : float) -> int:
        slope = (self.encoder_extend - self.encoder_retract) / (self.length_extend - self.length_retract)
        
        return int(slope * (length - self.length_retract) + self.encoder_retract)


# Motor controller instance.
class Motor_Controller:
    
    def __init__(self, rc: Roboclaw, address: int, m1: Encoder, m2: Encoder):
        self.rc = rc
        self.address = address
        self.m1 = m1
        self.m2 = m2 

        self.rc.Open()

    # Print speed and encoder values of both motors to terminal
    def print_encoders(self) -> None:
        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected")
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")


# Set position of arm using PID function.
def set_arm_position(mcp: Motor_Controller, shoulderJointAngle : float, elbowJointAngle : float, baseRotationAngle) -> None:
    
    encoder_val_m1 = mcp.m1.angle_to_enc(elbowJointAngle)
    encoder_val_m2 = mcp.m2.angle_to_enc(shoulderJointAngle)

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        1  # 0 for buffer, 1 for instant write
    )

    print(shoulderJointAngle, " ", elbowJointAngle, " ", baseRotationAngle)


#NOTE for testing of angle_to_length function
def main():
    # Create instance of a motor controller.
    mcp1 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),
        address = 0x80,  
        m1 = Encoder(  # elbow joint
            encoder_extend = 20, #20
            encoder_retract = 1930, # 1930

            angle_extend = 75,
            angle_retract = 140,

            length_extend = 13.47,  # inches
            length_retract = 10.03927072,  # inches

            actuator_pos = LINEAR_ACTUATOR_POS(3.0, 1.125, 12.50719421)
        ),
        m2 = Encoder(  # shoulder joint
            encoder_extend = 30, #30
            encoder_retract = 2640, #2640
            
            angle_extend = 5,
            angle_retract = 75,

            length_extend = 13.47,  # inches
            length_retract = 9.53,  # inches

            actuator_pos = LINEAR_ACTUATOR_POS(7.16717277, 1.0, 6.5)
        )
    )

    angle = float(sys.argv[1]) if len(sys.argv) > 1 else 0
    length = mcp1.m2.angle_to_length(angle)
    encoder = mcp1.m2.length_to_enc(length)
    print(f"angle: {angle}")
    print(f"length: {length}")
    print(f"encoder: {encoder}")


if __name__=='__main__':
    #main()
    ...

# rm -rf build install log && colcon build && source ./install/setup.bash && ros2 run py_pubsub listener