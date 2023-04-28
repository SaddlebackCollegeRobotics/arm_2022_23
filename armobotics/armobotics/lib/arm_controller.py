from dataclasses import dataclass
import math
import sys

# PATH_TO_ROBOCLAW = "../../lib/roboclaw_python"
# sys.path.append(PATH_TO_ROBOCLAW)

from roboclaw_3 import Roboclaw
from collections import namedtuple

BUFFER_OR_INSTANT = 1  # 0 for buffer, 1 for instant write

# TODO - Eventually change to be put into MotorController declarations

SPEED = 850
ACCELERATION = 700

ROLL_SPEED = 3000
ROLL_ACCEL = 2500

PITCH_SPEED = 3000
PITCH_ACCEL = 2500

TURRET_SPEED = 850
TURRET_ACCEL = 700

# linear actuator 
# d (inches) - distance from joint of arm to linear actuator
# h (inches) - vertical height between linear actuator base and arm hinge
# l (inches) - horizontal length between linear actuator base and arm hinge
actuator_tri = namedtuple('actuator_tri', 'a b theta')

@dataclass(frozen=True)
class Angle:
    angle_min : float
    angle_max : float

@dataclass(frozen=True)
class Encoder:
    encoder_max : int
    encoder_min : int

@dataclass(frozen=True)
class Linear_Actuator(Encoder, Angle):
    length_min : float 
    length_max : float 
    position_on_arm : actuator_tri 

@dataclass(frozen=True)
class Rotation_Motor(Encoder, Angle):
    ...

@dataclass(frozen=True)
class Gripper_Motor():
    ...

class Motor_Controller:
    def __init__(self, rc: Roboclaw, address: int, m1: Encoder, m2: Encoder):
        self.rc, self.address = rc, address
        self.m1, self.m2 = m1, m2

        self.rc.Open()

    # Print speed and encoder values of both motors to terminal
    def print_encoders(self) -> None:
        enc1 = self.rc.ReadEncM1(self.address)
        print(f"M1: {enc1[1]}" if enc1[0] == 1 else "disconnected", end=' ')
        enc2 = self.rc.ReadEncM2(self.address)
        print(f"M2: {enc2[1]}" if enc2[0] == 1 else "disconnected")



#NOTE correctly maps angle and encoder values at limits, but loses accuracy the closer it gets to center
# Convert angle to encoder value taking into account min/max encoder values and angle values.
def angle_to_enc(motor : Linear_Actuator or Rotation_Motor, angle : float) -> int:
    slope = (motor.encoder_max - motor.encoder_min) / (motor.angle_max - motor.angle_min)
    
    return int(slope * (angle - motor.angle_min) + motor.encoder_min)

#NOTE use encoder.angle_to_enc for now, angle_to_length not currently working
# Convert angle linear actuator lengths taking into account min/max angles and length values.
def angle_to_length(motor : Linear_Actuator, angle : float) -> float:
    a, b, theta = motor.position_on_arm 
    
    # Law of cosines
    actuator_length = math.sqrt(math.pow(a, 2) + math.pow(b, 2) - (2 * a * b * math.cos(180 - angle - theta)))

    return actuator_length

#NOTE correctly maps between length and encoder values
# Convert linear actuator lengths to encoder value taking into account min/max lengths and angle values.
def length_to_enc(motor : Linear_Actuator, length : float) -> int:
    slope = (motor.encoder_max - motor.encoder_min) / (motor.length_max - motor.length_min)
    
    return int(slope * (length - motor.length_min) + motor.encoder_min)



# Set position of arm using PID function.
def set_arm_position(mcp: Motor_Controller, bicep_angle: float, forearm_angle: float) -> None:

    encoder_val_m1 = angle_to_enc(mcp.m1, bicep_angle)
    #encoder_val_m1 = length_to_enc(mcp.m1, angle_to_length(mcp.m1, bicep_angle))
    encoder_val_m2 = angle_to_enc(mcp.m2, forearm_angle)
    

    # encoder_val_m1 = length_to_enc(mcp.m1, angle_to_length(mcp.m1, bicep_angle))
    # encoder_val_m2 = length_to_enc(mcp.m2, angle_to_length(mcp.m2, forearm_angle))

    # print(bicep_angle, " ", encoder_val_m1, " ", )

    # print(encoder_val_m1, " ", bicep_angle, " ", encoder_val_m2, " ", forearm_angle)    

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )

def set_arm_rotation(mcp: Motor_Controller, base_angle : float) -> None:
    encoder_val = angle_to_enc(mcp.m2, base_angle)
    #encoder_val = length_to_enc(mcp.m1, mcp.m1.angle_to_length(mcp.m1, base_angle))

    mcp.rc.SpeedAccelDeccelPositionM2(mcp.address, 
        TURRET_ACCEL, TURRET_SPEED, TURRET_ACCEL, encoder_val, BUFFER_OR_INSTANT
    )


def set_hand_rotation(mcp: Motor_Controller, hand_pitch: float, hand_roll: float) -> None:

    encoder_val_m1 = angle_to_enc(mcp.m1, hand_roll)
    encoder_val_m2 = angle_to_enc(mcp.m2, hand_pitch)
    #encoder_val_m1 = length_to_enc(mcp.m1, angle_to_length(mcp.m1, hand_pitch))
    #encoder_val_m2 = length_to_enc(mcp.m2, angle_to_length(mcp.m2, hand_roll))

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ROLL_ACCEL, ROLL_SPEED, ROLL_ACCEL, encoder_val_m1, 
        PITCH_ACCEL, PITCH_SPEED, PITCH_ACCEL, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )


def open_close_hand(mcp: Motor_Controller, move_velocity):
    # encoder_val = mcp.rc.ReadEncM1(mcp.address)

    if move_velocity == 0:
        mcp.rc.ForwardM1(mcp.address, 0)
    else:
        # mcp.rc.SpeedAccelM1(mcp.address, 20, int(move_velocity))
        mcp.rc.SpeedAccelM1(mcp.address, 20, int(move_velocity))

# rm -rf build install log && colcon build && source ./install/setup.bash && ros2 run py_pubsub listener