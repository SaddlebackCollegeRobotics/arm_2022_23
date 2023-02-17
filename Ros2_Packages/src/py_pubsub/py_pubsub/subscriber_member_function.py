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
        # Create instance of a motor controller.
        self.mcp1 = Motor_Controller(
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

        # Give the node a name.
        super().__init__('minimal_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Arm_Controls_Sim',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    # This callback definition simply prints an info message to the console, along with the data it received. 
    def listener_callback(self, msg):
        # print(msg.data[0], " ", msg.data[1], " ", msg.data[2])
        arm_controller.set_arm_position(self.mcp1, msg.data[0], msg.data[1], msg.data[2])
        


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
