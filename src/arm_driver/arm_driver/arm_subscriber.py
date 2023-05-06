# Author: Cameron Wolff - @camwolff02, Cameron Rosenthal - @Supernova1114

# Ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# General imports
from time import perf_counter
from .arm_controller import *
from .small_linear_actuator import Alfreds_Finger
import signal
import subprocess
import os
import sys


class ArmDriver(Node):

    # Quit program safely
    def quit_program_safely(self):

        self.softStop()
        # self.poker_controller.cleanup()

        print("\nExited Safely")

        sys.exit()


    # Callback for Ctrl+C
    def signalHandler(self, signal, frame):
        self.quit_program_safely()


    def __init__(self):

        self.angle_roll : float = 0.0

        # Signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signalHandler)

        # Get motor controller device paths
        self.mcp_comport_list = self.get_motor_controllers()

        if self.mcp_comport_list.__len__() == 0:
            print("No motor controllers detected. Exiting.")
            sys.exit()

        # Initialize motor controllers
        self.initialize_motor_controllers()
        self.initialize_additional_controllers()

        # Give the node a name.
        super().__init__('arm_driver')

        # Ask to run open or closed loop
        self.driver_mode = self.show_menu()

        if self.driver_mode == 1: 

            # Non IK controls subscription
            self.subscription = self.create_subscription(
                Float32MultiArray,
                '/arm/control_instruction_nonIK',
                self.nonIK_subscriber_callback,  # subscriber callback
                10)
            self.subscription  # prevent unused variable warning

        elif self.driver_mode == 2:

            # IK controls subscription
            self.subscription = self.create_subscription(
                Float32MultiArray,
                '/arm/control_instruction_IK',
                self.IK_subscriber_callback,  # subscriber callback
                10)
            self.subscription  # prevent unused variable warning

        self.time_of_last_callback = perf_counter()
        self.timer_period = 0.5 # Interval for checking heartbeat.
        self.signal_timeout = 3 # Max time before soft stop if subscription heartbeat is not detected.
        self.timer = self.create_timer(self.timer_period, self.subscription_heartbeat)


    # Show menu on program start
    def show_menu(self):

        MIN_MENU = 1
        MAX_MENU = 2

        while True:

            # Choose between IK and Normal subscriber
            choice = input("Choose an option:\n1. Normal\n2. IK\n:: ")

            if choice.isnumeric() == False:
                continue

            choice = int(choice)

            if choice < MIN_MENU or choice > MAX_MENU:
                continue

            return choice

            
    # For safety. Check if controls are being received by subscriber callback
    # Stop motors if subscription not being received.
    def subscription_heartbeat(self):
        if perf_counter() - self.time_of_last_callback > self.signal_timeout:
            self.softStop()


    # Signal to stop all motors
    def softStop(self):
        
        print("Soft Stop Triggered")

        # Clean up GPIO pins for poker
        self.poker.cleanup()

        for mcp in self.MCP_List:
            if mcp is not None:
                mcp.rc.ForwardM1(mcp.address, 0)
                mcp.rc.ForwardM2(mcp.address, 0)


    # Arm control instructions for IK control
    # Called every time the subscriber receives a message
    def IK_subscriber_callback(self, msg):

        self.time_of_last_callback = perf_counter()

        # Unpack ROS2 message
        bicep_actuator_len, forearm_actuator_len = msg.data[0], msg.data[1]
        turret_angle = msg.data[2]
        pitch_angle, roll_dir = msg.data[3], msg.data[4]
        grip_velocity = msg.data[5]
        poker_velocity = msg.data[6]
        safety_trigger = True if int(msg.data[7]) == 1 else False

        # Handle safety trigger
        if safety_trigger == False:
            self.softStop()
            return

        print("Arm control instructions received.")

        # Bicep and forearm
        set_arm_position(self.mcp2, bicep_actuator_len, forearm_actuator_len)

        # Turret rotation
        set_arm_rotation(self.mcp3, turret_angle)

        # End-effector pitch
        set_hand_pitch(self.mcp1, pitch_angle)

        # Temp set to non PID as encoder wires need fixing.
        # End-effector roll
        self.angle_roll = set_hand_roll_velocity(self.mcp1, int(roll_dir), self.angle_roll)

        # Grip movement
        open_close_hand(self.mcp3, int(grip_velocity))

        # Poker movement

        print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')


    # Arm control instructions for non IK control
    # Called every time the subscriber receives a message
    def nonIK_subscriber_callback(self, msg):
        
        # ******************************************************************************
        # * UNTIL IMPLEMENTED, REMOVE WHEN DONE
        # ******************************************************************************
        assert len(msg.data) != 8, "ADD 2 MORE BUTTON FOR PUSH AND PULL FOR POKER, PLEASE ADD"

        # ******************************************************************************
        # * UNTIL IMPLEMENTED, REMOVE WHEN DONE
        # ******************************************************************************


        self.time_of_last_callback = perf_counter()

        # Unpack ROS2 message
        bicep_dir, forearm_dir = msg.data[0], msg.data[1]
        turret_dir = msg.data[2]
        pitch_dir, roll_dir = msg.data[3], msg.data[4]
        grip_velocity = msg.data[5]
        poker_velocity = msg.data[6]
        safety_trigger = True if int(msg.data[7]) == 1 else False

        # Handle safety trigger
        if safety_trigger == False:
            self.softStop()
            return

        # Bicep and forearm
        set_arm_velocity(self.mcp2, int(bicep_dir), int(forearm_dir))

        # Turret rotation
        set_arm_rotation_velocity(self.mcp3, int(turret_dir))

        # End-effector pitch and roll
        set_hand_rotation_velocity(self.mcp1, int(pitch_dir), int(roll_dir))

        # Grip movement
        open_close_hand(self.mcp3, int(grip_velocity))


        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # ! Change the msg.data[6] & msg.data[8] to be pressable button inputs like 0 & 1 !
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # Poker movement
        self.set_poker_movement(msg.data[6], msg.data[8])

        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # ! Change the msg.data[6] & msg.data[8] to be pressable button inputs like 0 & 1 !
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



    # Get motor controller device paths using serial IDs
    def get_motor_controllers(self):

        getter_script = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../share/arm_driver/find_devpath.bash')
        device_list = subprocess.run(["\"" + getter_script + "\""], stdout=subprocess.PIPE, text=True, shell=True, executable='/bin/bash').stdout.splitlines()
        
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
            
        # motor controller for end-effector pitch and roll
        self.mcp1 = MotorController(
            rc = Roboclaw(self.mcp_comport_list[0], 115200),
            address = 0x80,  

            # Roll
            m1 = RotationMotor( 
                encoder_max = 6580,  # Max PID position
                encoder_min = -6583, # Min PID position
                angle_max   = 90,    # retract
                angle_min   = -90,   # extend
            ),

            # Pitch
            m2 = RotationMotor(
                encoder_max = 7656,  # Max PID position
                encoder_min = -6722, # Min PID position
                angle_max   = -90,   # retract
                angle_min   = 90,    # extend
            )
        )

        # motor controller for forearm and bicep linear actuators
        self.mcp2 = MotorController(
            rc = Roboclaw(self.mcp_comport_list[1], 115200),
            address = 0x80,  

            # Bicep
            m1 = LinearActuator(
                encoder_max = 2633,   # retract
                encoder_min = 153,    # extend
                angle_max   = 75,     # retract
                angle_min   = 5,      # extend
                length_min  = 4.1057, # extend, units
                length_max  = 2.8874, # retract, units
            ),

            # Forearm
            m2 = LinearActuator(
                encoder_max = 1893,   # retract
                encoder_min = 20,     # extend
                angle_max   = 140,    # retract
                angle_min   = 75,     # extend
                length_min  = 4.0653, # extend, units
                length_max  = 3.1037, # retract, units
            )
        )

        # Motor controller for turret and grip
        self.mcp3 = MotorController(
            rc = Roboclaw(self.mcp_comport_list[2], 115200),
            address = 0x80,  

            # Grip
            m1 = GripperMotor(
                # Note: Grip motor does not have encoder
            ),

            # Turret
            m2 = RotationMotor(
                angle_min = -120,
                angle_max = 120,
                encoder_max = 13993,
                encoder_min = -11314
            )
        )

        # Create list of motor controllers
        self.MCP_List = [self.mcp1, self.mcp2, self.mcp3]


    # Initialize additional motor controllers
    def initialize_additional_controllers(self):
        self.poker = Alfreds_Finger()
        
    
    def set_poker_movement(self, push_button: int, pull_button: int):
        if abs(push_button):
            self.poker.push()

        elif abs(pull_button):
            self.poker.pull()






def main(args=None):

    rclpy.init(args=args)

    arm_driver = ArmDriver()

    rclpy.spin(arm_driver)

    # Destroy the node explicitly
    arm_driver.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
