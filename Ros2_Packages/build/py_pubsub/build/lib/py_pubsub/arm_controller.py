from dataclasses import dataclass
import math
import sys

# PATH_TO_ROBOCLAW = "../../lib/roboclaw_python"
# sys.path.append(PATH_TO_ROBOCLAW)

from .roboclaw_3 import Roboclaw
# from roboclaw_3 import Roboclaw
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
    position_on_arm : actuator_pos 

@dataclass(frozen=True)
class Rotation_Motor(Encoder, Angle):
    ...

@dataclass(frozen=True)
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
    slope = (motor.encoder_max - motor.encoder_min) / (motor.angle_max - motor.angle_min)
    
    return int(slope * (angle - motor.angle_min) + motor.encoder_min)

#NOTE use encoder.angle_to_enc for now, angle_to_length not currently working
# Convert angle linear actuator lengths taking into account min/max angles and length values.
def angle_to_length(motor : Linear_Actuator, angle : float) -> float:
    d, h, l = motor.position_on_arm 
    actuator_length = math.sqrt( math.pow(l + d*math.cos(angle) , 2) + math.pow(h - d*math.sin(angle) , 2) )
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
    #encoder_val_m2 = length_to_enc(mcp.m1, angle_to_length(mcp.m1, forearm_angle))

    # print(encoder_val_m1, " ", encoder_val_m2)    

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )

def set_arm_rotation(mcp: Motor_Controller, base_angle : float) -> None:
    encoder_val = angle_to_enc(mcp.m1, base_angle)
    #encoder_val = length_to_enc(mcp.m1, mcp.m1.angle_to_length(mcp.m1, base_angle))

    mcp.rc.SpeedAccelDeccelPositionM1(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val, BUFFER_OR_INSTANT
    )

def set_hand_rotation(mcp: Motor_Controller, hand_pitch: float, hand_roll: float) -> None:
    encoder_val_m1 = angle_to_enc(mcp.m1, hand_pitch)
    #encoder_val_m1 = length_to_enc(mcp.m1, angle_to_length(mcp.m1, hand_pitch))
    encoder_val_m2 = angle_to_enc(mcp.m2, hand_roll)
    #encoder_val_m2 = length_to_enc(mcp.m2, angle_to_length(mcp.m2, hand_roll))

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m1, 
        ACCELERATION, SPEED, ACCELERATION, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )

def open_close_hand(mcp: Motor_Controller, move_velocity):
    encoder_val = mcp.rc.ReadEncM2(mcp.address)

    if move_velocity == 0:
        mcp.rc.ForwardM2(0)

    elif move_velocity > 0 and encoder_val < mcp.m2.encoder_max:
        mcp.rc.ForwardM2(move_velocity)
    
    elif move_velocity < 0 and encoder_val > mcp.m2.encoder_min:
        mcp.rc.BackwardM2(math.abs(move_velocity))



#NOTE for testing of angle_to_length function
def main():
    # motor controller with hand pitch and roll 
    mcp1 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),  #TODO figure out correct comport
        address = 0x0,  #TODO set val
        m1 = Rotation_Motor(  # pitch motor
            encoder_max = 0,  #TODO set value
            encoder_min = 0,  #TODO set value
            angle_min = -90,  
            angle_max = 90   
        ),
        m2 = Rotation_Motor(  # roll motor
            encoder_max = 0,  #TODO set value
            encoder_min = 0,  #TODO set value
            angle_min = -90,  
            angle_max = 90 
        )
    )

    # motor controller with forearm and bicep linear
    # motor controller with forearm and bicep linear
    mcp2 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),
        address = 0x80,  
        m1 = Linear_Actuator(  # bicep 
            encoder_max = 2633,      # retract
            encoder_min = 25,        # extend
            angle_max   = 75,        # retract
            angle_min   = 5,         # extend
            length_max  = 9.53,      # retract, inches
            length_min  = 13.47,     # extend, inches
            position_on_arm = actuator_pos(7.16717277, 1.0, 6.5)  # inches
        ),
        m2 = Linear_Actuator(  # forearm
            encoder_max = 1893,         # retract
            encoder_min = 20,           # extend
            angle_max   = 140,          # retract
            angle_min   = 75,           # extend
            length_max  = 10.03927072,  # retract, inches
            length_min  = 13.47,        # extend, inches
            position_on_arm = actuator_pos(3.0, 1.125, 12.50719421)  # inches
        )
    )

    # motor controller with hand pitch and roll motors 
    mcp3 = Motor_Controller(
        rc = Roboclaw(COMPORT_NAME, 115200),  #TODO figure out correct comport
        address = 0x0,  #TODO set val
        m1 = Rotation_Motor(  # pitch motor
            encoder_max = 0,  #TODO set value
            encoder_min = 0,  #TODO set value
            angle_min = -180,  
            angle_max = 180
        ),
        m2 = Gripper_Motor(  # roll motor
            encoder_max = 0,  #TODO set value
            encoder_min = 0   #TODO set value
        )
    )

    angle1 = float(sys.argv[1]) if len(sys.argv) > 1 else 45
    angle2 = float(sys.argv[2]) if len(sys.argv) > 2 else 45

    length1 = angle_to_length(mcp2.m1, angle1)
    length2 = angle_to_length(mcp2.m2, angle2)

    # encoder1 = length_to_enc(mcp2.m1, length1)
    # encoder2 = length_to_enc(mcp2.m2, length2)

    encoder1 = angle_to_enc(mcp2.m1, angle1)
    encoder2 = angle_to_enc(mcp2.m2, angle2)

    print(f"angles: {angle1}  {angle2}")
    #print(f"lengths: {length1}  {length2}")
    print(f"encoders: {encoder1}  {encoder2}")

    set_arm_position(mcp2, angle1, angle2)


if __name__=='__main__':
    main()
    

# rm -rf build install log && colcon build && source ./install/setup.bash && ros2 run py_pubsub listener