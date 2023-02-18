from dataclasses import dataclass
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

BUFFER_OR_INSTANT = 1  # 0 for buffer, 1 for instant write

SPEED = 850 #600
ACCELERATION = 700 #200

# linear actuator position tuple
# d (inches) - distance from joint of arm to linear actuator
# h (inches) - vertical height between linear actuator base and arm hinge
# l (inches) - horizontal length between linear actuator base and arm hinge
actuator_pos = namedtuple('actuator_pos', 'd h l')

@dataclass 
class Angle:
    ANGLE_MIN : float
    ANGLE_MAX : float

@dataclass 
class Encoder:
    ENCODER_MAX : int
    ENCODER_MIN : int

@dataclass
class Linear_Actuator(Encoder, Angle):
    LENGTH_MIN : float 
    LENGTH_MAX : float 
    POSITION_ON_ARM : actuator_pos 

@dataclass 
class Rotation_Motor(Encoder, Angle):
    ...

@dataclass 
class Gripper_Motor(Encoder):
    ...

class Motor_Controller:
    def __init__(self, rc: Roboclaw, address: int, m1: Encoder, m2: Encoder):
        self.rc, self.address = rc, address
        self.m1, self.m2 = m1, m2

        self.rc.Open()

    # Print speed and encoder values of both motors to terminal
    def print_encoders(self) -> None:
        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected")
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")



#NOTE correctly maps angle and encoder values at limits, but loses accuracy the closer it gets to center
# Convert angle to encoder value taking into account min/max encoder values and angle values.
def angle_to_enc(motor : Linear_Actuator or Rotation_Motor, angle : float) -> int:
    slope = (motor.ENCODER_MAX - motor.ENCODER_MIN) / (motor.ANGLE_MAX - motor.ANGLE_MIN)
    
    return int(slope * (angle - motor.ANGLE_MIN) + motor.ENCODER_MIN)

#NOTE use encoder.angle_to_enc for now, angle_to_length not currently working
# Convert angle linear actuator lengths taking into account min/max angles and length values.
def angle_to_length(motor : Linear_Actuator, angle : float) -> float:
    d, h, l = motor.POSITION_ON_ARM 
    actuator_length = math.sqrt( math.pow(l + d*math.cos(angle) , 2) + math.pow(h - d*math.sin(angle) , 2) )
    return actuator_length

#NOTE correctly maps between length and encoder values
# Convert linear actuator lengths to encoder value taking into account min/max lengths and angle values.
def length_to_enc(motor : Linear_Actuator, length : float) -> int:
    slope = (motor.ENCODER_MAX - motor.ENCODER_MIN) / (motor.LENGTH_MAX - motor.LENGTH_MIN)
    
    return int(slope * (length - motor.LENGTH_MIN) + motor.ENCODER_MIN)



# Set position of arm using PID function.
def set_arm_position(mcp: Motor_Controller, shoulder_angle: float, elbow_angle: float) -> None:
    
    encoder_val_m1 = mcp.m1.angle_to_enc(elbow_angle)
    #encoder_val_m1 = mcp.m1.length_to_enc( mcp.m1.angle_to_length(elbow_angle))
    encoder_val_m2 = mcp.m2.angle_to_enc(shoulder_angle)
    #encoder_val_m2 = mcp.m2.length_to_enc( mcp.m2.angle_to_length(shoulder_angle))

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )

def set_arm_rotation(mcp: Motor_Controller, base_angle : float) -> None:
    encoder_val_m1 = mcp.m1.angle_to_enc(base_angle)
    #encoder_val_m1 = mcp.m1.length_to_enc( mcp.m1.angle_to_length(base_angle))

    mcp.rc.SpeedAccelDeccelPositionM1(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, BUFFER_OR_INSTANT
    )

def set_hand_rotation(mcp: Motor_Controller, hand_pitch: float, hand_roll: float) -> None:
    encoder_val_m1 = mcp.m1.angle_to_enc(hand_pitch)
    #encoder_val_m1 = mcp.m1.length_to_enc( mcp.m1.angle_to_length(hand_pitch))
    encoder_val_m2 = mcp.m2.angle_to_enc(hand_roll)
    #encoder_val_m2 = mcp.m2.length_to_enc( mcp.m2.angle_to_length(hand_roll))

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )

def open_close_hand(mcp: Motor_Controller, move_velocity):
    encoder_val = mcp.rc.ReadEncM2(mcp.address)

    if move_velocity == 0:
        mcp.rc.ForwardM2(0)

    elif move_velocity > 0 and encoder_val < mcp.m2.ENCODER_MAX:
        mcp.rc.ForwardM2(move_velocity)
    
    elif move_velocity < 0 and encoder_val > mcp.m2.ENCODER_MIN:
        mcp.rc.BackwardM2(math.abs(move_velocity))



#NOTE for testing of angle_to_length function
def main():
    # motor controller with forearm and bicep linear
    mcp1 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),
        address = 0x80,  
        m1 = Linear_Actuator(  # elbow joint
            ENCODER_MAX = 20,           # extended
            ENCODER_MIN = 1930,         # retract
            ANGLE_MAX   = 75,           # extend
            ANGLE_MIN   = 140,          # retract
            LENGTH_MAX  = 13.47,        # extend, inches
            LENGTH_MIN  = 10.03927072,  # retract, inches
            POSITION_ON_ARM = actuator_pos(3.0, 1.125, 12.50719421)  # inches
        ),
        m2 = Linear_Actuator(  # shoulder joint
            ENCODER_MAX = 30,       # extended
            ENCODER_MIN = 2640,     # retract
            ANGLE_MAX   = 5,        # extend
            ANGLE_MIN   = 75,       # retract
            LENGTH_MAX  = 13.47,    # extend, inches
            LENGTH_MIN  = 9.53,     # retract, inches
            POSITION_ON_ARM = actuator_pos(7.16717277, 1.0, 6.5)  # inches
        )
    )

    # motor controller with arm rotation and hand grip
    mcp2 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),  #TODO figure out correct comport
        address = 0x0,  #TODO set val
        m1 = Rotation_Motor(  # pitch motor
            ENCODER_MAX = 0,  #TODO set value
            ENCODER_MIN = 0,  #TODO set value
            ANGLE_MIN = -90,  
            ANGLE_MAX = 90   
        ),
        m2 = Rotation_Motor(  # roll motor
            ENCODER_MAX = 0,  #TODO set value
            ENCODER_MIN = 0,  #TODO set value
            ANGLE_MIN = -90,  
            ANGLE_MAX = 90 
        )
    )

    # motor controller with hand pitch and roll motors 
    mcp3 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),  #TODO figure out correct comport
        address = 0x0,  #TODO set val
        m1 = Rotation_Motor(  # pitch motor
            ENCODER_MAX = 0,  #TODO set value
            ENCODER_MIN = 0,  #TODO set value
            ANGLE_MIN = -180,  
            ANGLE_MAX = 180
        ),
        m2 = Gripper_Motor(  # roll motor
            ENCODER_MAX = 0,  #TODO set value
            ENCODER_MIN = 0   #TODO set value
        )
    )

    angle1 = float(sys.argv[1]) if len(sys.argv) > 1 else 45
    angle2 = float(sys.argv[2]) if len(sys.argv) > 2 else 45

    length1 = angle_to_length(mcp1.m1, angle1)
    length2 = angle_to_length(mcp1.m2, angle2)

    # encoder1 = length_to_enc(mcp1.m1, length1)
    # encoder2 = length_to_enc(mcp1.m2, length2)

    encoder1 = angle_to_enc(mcp1.m1, angle1)
    encoder2 = angle_to_enc(mcp1.m2, angle2)

    print(f"angles: {angle1}  {angle2}")
    #print(f"lengths: {length1}  {length2}")
    print(f"encoders: {encoder1}  {encoder2}")


if __name__=='__main__':
    main()
    

# rm -rf build install log && colcon build && source ./install/setup.bash && ros2 run py_pubsub listener