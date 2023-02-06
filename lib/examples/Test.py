import time
from collections import namedtuple

from attr import dataclass
from roboclaw_3 import Roboclaw

# Windows comport name
# roboclaw = Roboclaw("COM8",115200)
# Linux comport name
# roboclaw = Roboclaw("/dev/ttyACM1",115200)
#MCP = namedtuple('rc', 'address')

@dataclass
class MCP:
    rc : Roboclaw
    address: int

ENCODER_MAX = 2635.0
ENCODER_MIN = 0.0

ANGLE_MAX = 90.0 - 15.47
ANGLE_MIN = 90.0 - 66.65

def print_encoders(mcp):
    enc = mcp.rc.ReadEncM2(mcp.address)
    print(f"encoder: {enc[1]}\n speed: {format(enc[2],'02x')}" if enc[0] == 1 else "failed")



def move_to_pos(mcp, target_encoder_val, speed=50, tolerance=50):
    if target_encoder_val < mcp.rc.ReadEncM2(mcp.address)[1]:
        mcp.rc.BackwardM2(mcp.address, speed)
    else:
        mcp.rc.ForwardM2(mcp.address, speed)

    while abs(mcp.rc.ReadEncM2(mcp.address)[1] - target_encoder_val) > tolerance:
        continue

    mcp.rc.ForwardM2(mcp.address, 0)



def angle_to_enc(angle):
    slope = (ENCODER_MIN - ENCODER_MAX) / (ANGLE_MIN - ANGLE_MAX)
    return slope * (angle - ANGLE_MAX) + ENCODER_MAX



def main():
    mcp = MCP(Roboclaw("/dev/ttyACM1",115200), 0x80)
    mcp.rc.Open()

    print_encoders(mcp)
    move_to_pos(mcp, angle_to_enc(45))
    time.sleep(100)
    print_encoders(mcp)

    # rc.BackwardM2(address,30)	#1/4 power backward
    # time.sleep(3)
    # rc.BackwardM2(address,30)
    # rc.SetM2PositionPID(address, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0)

if __name__ == '__main__':
    main()