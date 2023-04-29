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
import subprocess
import os


class MinimalSubscriber(Node):

    # Quit program safely
    def quit_program_safely(self):

        print("\nExited Safely")
        self.SoftStop()
        quit()


    # Callback for Ctrl+C
    def signalHandler(self, signal, frame):
        self.quit_program_safely()


    def __init__(self):

        # Signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signalHandler)


        # Get motor controller device paths
        self.mcp_comport_list = self.get_motor_controllers()


        # TODO - need to have these motors use PID position rather than encoder values? DO for mcp3 as well
        # May want to do this for lin actuators as well. need to test 
        
        # Initialize motor controllers
        # try:
        self.initialize_motor_controllers()
        

        # TODO - Eventually save previously known value for quad encoders. Do we need this for now?
        # Set encoder values to zero
        # self.mcp1.rc.SetEncM1(self.mcp1.address, 0) # Roll
        # self.mcp1.rc.SetEncM2(self.mcp1.address, 0) # Pitch
        # self.mcp3.rc.SetEncM2(self.mcp3.address, 0) # Turret

        # Give the node a name.
        super().__init__('minimal_subscriber')

        # Subscribe to the topic 'topic'. Callback gets called when a message is received.
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Arm_Controls_Sim',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


        self.time_of_last_callback = perf_counter()
        self.timer_period = 0.1 # Interval for checking heartbeat.
        self.signal_timeout = 3 # Max time before soft stop if subscription heartbeat is not detected.
        self.timer = self.create_timer(self.timer_period, self.Subscription_HeartBeat)


    # For safety. Check if controls are being received by subscriber callback
    # Stop motors if subscription not being received.
    def Subscription_HeartBeat(self):
        if perf_counter() - self.time_of_last_callback > self.signal_timeout:
            self.SoftStop()


    # Signal to stop all motors
    def SoftStop(self):
        
        print("Soft Stop Triggered")

        for mcp in self.MCP_List:
            if mcp is not None:
                mcp.rc.ForwardM1(mcp.address, 0)
                mcp.rc.ForwardM2(mcp.address, 0)


    # called every time the subscriber receives a message
    def listener_callback(self, msg):

        self.time_of_last_callback = perf_counter()

        bicep_angle, forearm_angle, base_angle = msg.data[0], msg.data[1], msg.data[2]
        pitch_angle, roll_angle, finger_velocity = msg.data[3], msg.data[4], msg.data[5]
        
        # only change the arm position if any angles have changed
        set_arm_position(self.mcp2, bicep_angle, forearm_angle)

        set_arm_rotation(self.mcp3, base_angle)

        # Hand pitch and roll
        set_hand_rotation(self.mcp1, pitch_angle, roll_angle)

        # Finger movement
        open_close_hand(self.mcp3, finger_velocity)

        # This prints an info message to the console, along with the data it received. 
        # for x in msg.data: print(x, end=' ')
        # print()


    # Get motor controller device paths using serial IDs
    # Returns in order (0001, 0002, 0003)
    def get_motor_controllers(self):

        # Bash script is not moving during colcon build, and so am just pasting it here.
        # TODO - Fix this

        getter_script = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../share/py_pubsub/find_devpath.bash')

        device_list = subprocess.run([getter_script], stdout=subprocess.PIPE, text=True, shell=True).stdout.splitlines()

        devpath_list = ["", "", ""]

        # Add device paths to devpath list
        for device in device_list:
            splitStr = device.split(" - ")

            if "ID0001" in splitStr[1]:
                devpath_list[0] = splitStr[0]

            elif "ID0002" in splitStr[1]:
                devpath_list[1] = splitStr[0]

            elif "ID0003" in splitStr[1]:
                devpath_list[2] = splitStr[0]

        return devpath_list
        

    # Initialize motor controller objects
    def initialize_motor_controllers(self):
            
            self.MCP_List = [None, None, None]

            # motor controller for end-effector pitch and roll
            self.mcp1 = Motor_Controller(
                rc = Roboclaw(self.mcp_comport_list[0], 115200),
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
                rc = Roboclaw(self.mcp_comport_list[1], 115200),
                address = 0x80,  
                m1 = Linear_Actuator(  # bicep 
                    encoder_max = 2633,      # retract
                    encoder_min = 153,        # extend
                    angle_max   = 75,        # retract
                    angle_min   = 5,         # extend
                    length_max  = 34.2138,   # extend, centimeters
                    length_min  = 24.2062,   # retract, centimeters
                    # position_on_arm = actuator_pos(7.16717277, 1.0, 6.5)  # inches
                    position_on_arm = actuator_tri(16.7042, 18.2046, 8.75) # (cm, cm, deg)
                ),
                m2 = Linear_Actuator(  # forearm
                    encoder_max = 1893,         # retract
                    encoder_min = 20,           # extend
                    angle_max   = 140,          # retract
                    angle_min   = 75,           # extend
                    length_max  = 34.2138,      # extend, centimeters
                    length_min  = 25.4762,      # retract, centimeters
                    # position_on_arm = actuator_tri(3.0, 1.125, 12.50719421)  # inches
                    position_on_arm = actuator_tri(31.8965, 7.62, 5.14) # (cm, cm, deg)
                )
            )


            # Motor controller for turret and grip
            self.mcp3 = Motor_Controller(
                rc = Roboclaw(self.mcp_comport_list[2], 115200),
                address = 0x80,  
                m1 = Gripper_Motor(  # Grip 
                    # Note: Grip motor does not have encoder
                ),
                m2 = Rotation_Motor(  # Turret
                    # Note: No limit needed for turret. Rotate via velocity rather than position.
                    angle_min = -120,
                    angle_max = 120,
                    encoder_max = 13993,
                    encoder_min = -11314
                )
            )

            # Create list of motor controllers
            self.MCP_List = [self.mcp1, self.mcp2, self.mcp3]




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
