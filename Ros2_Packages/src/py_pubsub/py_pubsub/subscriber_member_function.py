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
from time import perf_counter
from .arm_controller import *
import signal






class MinimalSubscriber(Node):

        # Callback for Ctrl+C
    def signalHandler(self, signal, frame):
        # self.mcp2.rc.ForwardM1(self.mcp2.address, 0)
        # self.mcp2.rc.ForwardM2(self.mcp2.address, 0)
        print("\nExited Cleanly")
        quit()

    def __init__(self):

        # Signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signalHandler)


        # TODO - need to have these motors use PID position rather than encoder values.
        # May want to do this for lin actuators as well. need to test 
        
        # motor controller for end-effector pitch and roll
        self.mcp1 = Motor_Controller(
            rc = Roboclaw(COMPORT_NAME_2, 115200),
            address = 0x80,  
            m1 = Rotation_Motor(  # roll 
                encoder_max = 6580,      # Max PID position
                encoder_min = -6583,        # Min PID position
                angle_max   = 90,        # retract
                angle_min   = -90,         # extend
            ),
            m2 = Rotation_Motor(  # pitch
                encoder_max = 7656,         # Max PID position
                encoder_min = -6722,           # Min PID position
                angle_max   = 90,          # retract
                angle_min   = -90,           # extend
            )
        )


        # motor controller for forearm and bicep linear actuators
        self.mcp2 = Motor_Controller(
            rc = Roboclaw(COMPORT_NAME_1, 115200),
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

        # TODO - temp set quad encs to zero
        self.mcp1.rc.SetEncM1(self.mcp1.address, 0)
        self.mcp1.rc.SetEncM2(self.mcp1.address, 0)

        #mcp3 = Motor_Controller(...)

        # remember previous positions
        self.bicep_angle, self.forearm_angle, self.base_angle = (-1,-1,-1)
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

        # self.time_of_last_callback = perf_counter()
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.doomsday)

    def doomsday(self):
        # if perf_counter() - self.time_of_last_callback > 1:  # doomsday triggers at 1 second
            # self.mcp2.rc.ForwardM1(self.mcp2.address, 0)
            # self.mcp2.rc.ForwardM2(self.mcp2.address, 0)
            # TODO - May not need doomsday func anymore
            ...

    # called every time the subscriber receives a message
    def listener_callback(self, msg):
        # self.time_of_last_callback = perf_counter()

        bicep_angle, forearm_angle, base_angle = msg.data[0], msg.data[1], msg.data[2]
        pitch_angle, roll_angle, finger_velocity = msg.data[3], msg.data[4], msg.data[5]
        
        # only change the arm position if any angles have changed
        set_arm_position(self.mcp2, bicep_angle, forearm_angle)

        #set_arm_rotation(self.mcp..., base_angle)

        print(pitch_angle, roll_angle)
        set_hand_rotation(self.mcp1, pitch_angle, roll_angle)

        # TODO control finger moevement
        #arm_controller.set_arm_position(self.mcp3, finger_velocity)

        # This prints an info message to the console, along with the data it received. 
        # for x in msg.data: print(x, end=' ')
        # print()
        


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
