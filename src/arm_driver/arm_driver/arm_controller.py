# Author: Cameron Wolff - @camwolff02, Cameron Rosenthal - @Supernova1114

from dataclasses import dataclass
from .roboclaw_3 import Roboclaw
# from .small_linear_actuator import Alfreds_Finger
# from collections import namedtuple


BUFFER_OR_INSTANT = 1  # 0 for buffer, 1 for instant write

# TODO - Eventually change to be put into MotorController declarations

BICEP_SPEED = 850
BICEP_ACCELERATION = 700

FOREARM_SPEED = 850
FOREARM_ACCELERATION = 700

ROLL_SPEED = 3000
ROLL_ACCEL = 2500

PITCH_SPEED = 3000
PITCH_ACCEL = 2500

TURRET_SPEED = 850
TURRET_ACCEL = 700

# Dataclass wrappers for writing arm data to motors ---------------------------------------------------------

@dataclass(frozen=True)
class Angle:
    angle_min : float
    angle_max : float

@dataclass(frozen=True)
class Encoder:
    encoder_max : int
    encoder_min : int

@dataclass(frozen=True)
class LinearActuator(Encoder, Angle):
    length_min : float 
    length_max : float 

@dataclass(frozen=True)
class RotationMotor(Encoder, Angle):
    ...

@dataclass(frozen=True)
class GripperMotor():
    ...


# Wrapper class for Roboclaw Motor Controller interfacing
class MotorController:

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


# Data conversion functions ---------------------------------------------------------------------------------

# Convert angle to encoder value
def angle_to_enc(motor : LinearActuator or RotationMotor, angle : float) -> int:

    """Convert angle to encoder value taking into account min/max encoder values and angle values.
    uses linear approximation of true encoder values, real implementations should be exponential"""

    slope = (motor.encoder_max - motor.encoder_min) / (motor.angle_max - motor.angle_min)
    
    return int(slope * (angle - motor.angle_min) + motor.encoder_min)


# Convert length to encoder value
def length_to_enc(motor : LinearActuator, length : float) -> int:

    """Convert linear actuator lengths to encoder value 
    taking into account min/max lengths and encoder values"""

    slope = (motor.encoder_max - motor.encoder_min) / (motor.length_max - motor.length_min)
    
    return int(slope * (length - motor.length_min) + motor.encoder_min)


# Command motor controllers ---------------------------------------------------------------------------------

# Set bicep and forearm position using PID
def set_arm_position(mcp: MotorController, bicep_actuator_len: float, forearm_actuator_len: float) -> None:

    encoder_val_m1 = length_to_enc(mcp.m1, bicep_actuator_len)
    encoder_val_m2 = length_to_enc(mcp.m2, forearm_actuator_len)

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        BICEP_ACCELERATION, BICEP_SPEED, BICEP_ACCELERATION, encoder_val_m1, 
        FOREARM_ACCELERATION, FOREARM_SPEED, FOREARM_ACCELERATION, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )


# Set turret rotation angle using PID
def set_arm_rotation(mcp: MotorController, base_angle : float) -> None:

    encoder_val = angle_to_enc(mcp.m2, base_angle)

    mcp.rc.SpeedAccelDeccelPositionM2(mcp.address, 
        TURRET_ACCEL, TURRET_SPEED, TURRET_ACCEL, encoder_val, BUFFER_OR_INSTANT
    )


# Set end-effector pitch and roll rotation angle using PID
def set_hand_rotation(mcp: MotorController, hand_pitch: float, hand_roll: float) -> None:

    encoder_val_m1 = angle_to_enc(mcp.m1, hand_roll)
    encoder_val_m2 = angle_to_enc(mcp.m2, hand_pitch)

    mcp.rc.SpeedAccelDeccelPositionM1M2(mcp.address, 
        ROLL_ACCEL, ROLL_SPEED, ROLL_ACCEL, encoder_val_m1, 
        PITCH_ACCEL, PITCH_SPEED, PITCH_ACCEL, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )


# Set end-effector roll using PID
def set_hand_roll(mcp: MotorController, hand_roll: float) -> None:

    encoder_val_m1 = angle_to_enc(mcp.m1, hand_roll)

    mcp.rc.SpeedAccelDeccelPositionM1(mcp.address, 
        ROLL_ACCEL, ROLL_SPEED, ROLL_ACCEL, encoder_val_m1, 
        BUFFER_OR_INSTANT 
    )


# Set end-effector pitch using PID
def set_hand_pitch(mcp: MotorController, hand_pitch: float) -> None:

    encoder_val_m2 = angle_to_enc(mcp.m2, hand_pitch)

    mcp.rc.SpeedAccelDeccelPositionM2(mcp.address, 
        PITCH_ACCEL, PITCH_SPEED, PITCH_ACCEL, encoder_val_m2, 
        BUFFER_OR_INSTANT 
    )


# Set end-effector pitch velocity using PWM
def set_hand_pitch_velocity(mcp: MotorController, pitch_dir: int):
    
    if pitch_dir == 0:
        mcp.rc.ForwardM2(mcp.address, 0)
    else:
        mcp.rc.SpeedAccelM2(mcp.address, 20, -pitch_dir)


# Set end-effector roll velocity using PWM
def set_hand_roll_velocity(mcp: MotorController, roll_dir: int, angle: int) -> int:

    if angle >= 90 and roll_dir == 1:
        mcp.rc.ForwardM1(mcp.address, 0)
    
    elif angle <= -90 and roll_dir == -1:
        mcp.rc.ForwardM1(mcp.address, 0)
    
    elif roll_dir == 0:
        mcp.rc.ForwardM1(mcp.address, 0)

    elif roll_dir == 1 and angle < 90:
        mcp.rc.BackwardM1(mcp.address, 20)
        angle += 1

    elif roll_dir == -1 and angle > -90:
        mcp.rc.ForwardM1(mcp.address, 20)
        angle -= 1

    return angle


# Set arm bicep and forearm velocities using PWM
def set_arm_velocity(mcp: MotorController, pitch_dir: int, roll_dir: int):
    raise NotImplementedError
    

# Set turret rotation velocity using PWM
def set_arm_rotation_velocity(mcp: MotorController, turret_dir: int):
    raise NotImplementedError


# Set end-effector grip movement
def open_close_hand(mcp: MotorController, move_velocity: int):
    
    if move_velocity == 0:
        mcp.rc.ForwardM1(mcp.address, 0)
    else:
        mcp.rc.SpeedAccelM1(mcp.address, 20, -move_velocity)


# Set poker movement
# def set_poker(motor_driver: Alfreds_Finger, move_velocity: int):

#     if move_velocity == 1:
#         motor_driver.push()
#     elif move_velocity == -1:
#         motor_driver.pull()
    