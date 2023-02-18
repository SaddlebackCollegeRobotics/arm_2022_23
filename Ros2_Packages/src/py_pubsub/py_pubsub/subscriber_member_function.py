# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# General imports
import time
from . import arm_controller
from arm_controller import *

class MinimalSubscriber(Node):

    def __init__(self):
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

        mcp2 = Motor_Controller(...)
        mcp3 = Motor_Controller(...)

        # remember previous positions
        self.shoulder_angle, self.elbow_angle, self.base_angle = (-1,-1,-1)
        self.pitch_angle, self.roll_angle = (-1, -1)

        # Give the node a name.
        super().__init__('minimal_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Arm_Controls_Sim',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    # called every time the subscriber receives a message
    def listener_callback(self, msg):
        shoulder_angle, elbow_angle, base_angle = msg.data[0], msg.data[1], msg.data[2]
        pitch_angle, roll_angle, finger_velocity = msg.data[3], msg.data[4], msg.data[5]
        
        # only change the arm position if any angles have changed
        if (shoulder_angle != self.shoulder_angle or elbow_angle != self.elbow_angle):
            arm_controller.set_arm_position(self.mcp1, shoulder_angle, elbow_angle)
            self.shoulder_angle, self.elbow_angle = shoulder_angle, elbow_angle

        if (base_angle != self.base_angle):
            arm_controller.set_arm_rotation(self.mcp3, base_angle)
            self.base_angle = base_angle

        if (pitch_angle != self.pitch_angle or roll_angle != self.roll_angle):
            arm_controller.set_hand_rotation(self.mcp2, pitch_angle, roll_angle)
            self.pitch_angle, self.roll_angle = pitch_angle, roll_angle

        arm_controller.set_arm_position(self.mcp3, finger_velocity)

        # This prints an info message to the console, along with the data it received. 
        for x in msg.data: print(x, end=' ')
        print()
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
