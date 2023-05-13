# Author: Cameron Wolff - @camwolff02, Cameron Rosenthal - @Supernova1114

# Ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# General imports
from time import perf_counter
from .motor_driver import *
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

        # Initialize ROS2 node
        super().__init__('arm_driver')

        # Ask to run open or closed loop
        self.driver_mode = self.show_menu()

        if self.driver_mode == 1: 

            # Open loop controls subscription
            self.subscription = self.create_subscription(
                Float32MultiArray,
                '/arm/control_instruction',
                self.open_loop_subscriber_callback, # subscriber callback
                10)
            self.subscription # prevent unused variable warning

        elif self.driver_mode == 2:

            # IK controls subscription
            self.subscription = self.create_subscription(
                Float32MultiArray,
                '/arm/control_instruction',
                self.IK_subscriber_callback, # subscriber callback
                10)
            self.subscription # prevent unused variable warning

        elif self.driver_mode == 3:

            # PID speed controls subscription
            self.subscription = self.create_subscription(
                Float32MultiArray,
                '/arm/control_instruction',
                self.PID_speed_subscriber_callback, # subscriber callback
                10)
            self.subscription # prevent unused variable warning

        self.time_of_last_callback = perf_counter()
        self.timer_period = 0.5 # Interval for checking heartbeat.
        self.signal_timeout = 3 # Max time before soft stop if subscription heartbeat is not detected.
        self.timer = self.create_timer(self.timer_period, self.subscription_heartbeat)


    # Show menu on program start
    def show_menu(self):

        MIN_MENU = 1
        MAX_MENU = 3

        while True:

            # Choose between IK and Normal subscriber
            choice = input("Choose an option:\n1. Open loop\n2. IK\n3. PID speed\n:: ")

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

        for mcp in self.MCP_List:
            if mcp is not None:
                mcp.rc.ForwardM1(mcp.address, 0)
                mcp.rc.ForwardM2(mcp.address, 0)

        # self.poker_controller.stop()


    # Arm control instructions for IK control
    # Called every time the subscriber receives a message
    def IK_subscriber_callback(self, msg):

        self.time_of_last_callback = perf_counter()

        # Unpack ROS2 message
        bicep_actuator_len, forearm_actuator_len = msg.data[0], msg.data[1]
        turret_angle = msg.data[2]
        pitch_angle, roll_dir = msg.data[3], msg.data[4]
        grip_velocity = msg.data[5]
        poker_dir = msg.data[6]
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
        set_hand_roll_velocity(self.mcp1, int(roll_dir))

        # Grip movement
        open_close_hand(self.mcp3, int(grip_velocity))

        # Poker movement
        # set_poker(self.poker_controller, poker_dir)

        # Flush stdout
        sys.stdout.flush()


    # Arm control instructions for open loop control
    # Called every time the subscriber receives a message
    def open_loop_subscriber_callback(self, msg):

        self.time_of_last_callback = perf_counter()

        # Unpack ROS2 message
        bicep_actuator_dir, forearm_actuator_dir = msg.data[0], msg.data[1]
        turret_rotation_dir = msg.data[2]
        pitch_dir, roll_dir = msg.data[3], msg.data[4]
        grip_dir = msg.data[5]
        poker_dir = msg.data[6]
        safety_trigger = True if int(msg.data[7]) == 1 else False

        # Handle safety trigger
        if safety_trigger == False:
            self.softStop()
            return

        print("Arm control instructions received.")

        # Bicep movement
        set_bicep_velocity(self.mcp2, bicep_actuator_dir)

        # Forearm movement
        set_forearm_velocity(self.mcp2, forearm_actuator_dir)
        
        # Turret rotation
        set_turret_velocity(self.mcp3, turret_rotation_dir)

        # End-effector pitch
        set_hand_pitch_velocity(self.mcp1, int(pitch_dir))

        # End-effector roll
        set_hand_roll_velocity(self.mcp1, int(roll_dir))

        # Grip movement
        open_close_hand(self.mcp3, int(grip_dir))

        # Poker movement
        # set_poker(self.poker_controller, poker_dir)

        # Flush stdout
        sys.stdout.flush()


    # Arm control instructions for PID control
    # Called every time the subscriber receives a message
    def PID_speed_subscriber_callback(self, msg):

        self.time_of_last_callback = perf_counter()

        # Unpack ROS2 message
        bicep_actuator_dir, forearm_actuator_dir = msg.data[0], msg.data[1]
        turret_rotation_dir = msg.data[2]
        pitch_dir, roll_dir = msg.data[3], msg.data[4]
        grip_dir = msg.data[5]
        poker_dir = msg.data[6]
        safety_trigger = True if int(msg.data[7]) == 1 else False

        # Handle safety trigger
        if safety_trigger == False:
            self.softStop()
            return

        # print("Arm control instructions received.")

        set_arm_speed_PID(self.mcp2, -bicep_actuator_dir, -forearm_actuator_dir)

        # Turret rotation
        set_turret_velocity(self.mcp3, turret_rotation_dir)

        # End-effector pitch
        set_pitch_speed_PID(self.mcp1, int(pitch_dir))

        # enc = self.mcp1.rc.ReadEncM2(self.mcp1.address)[1]
        # newPos = enc + (int(-pitch_dir) * 1000)

        # self.mcp1.rc.SpeedAccelDeccelPositionM2(self.mcp1.address, 300, 300, 300, newPos, 1)

        # print(self.mcp1.rc.ReadEncM2(self.mcp1.address))

        # print(int(pitch_dir) * 300)
        # self.mcp1.rc.SpeedM2(self.mcp1.address, int(pitch_dir) * 300)
        # self.mcp3.rc.SpeedM2(self.mcp3.address, int(turret_rotation_dir) * 5)
        # print(int(turret_rotation_dir * 5))
        # self.mcp3.rc.SpeedAccelM2(self.mcp3.address, 300, int(turret_rotation_dir * 50))
        
        # self.mcp2.rc.SpeedAccelDeccelPositionM1M2(self.mcp2.address, 300, int(bicep_actuator_dir * 300), 300, 300, 300, 300, 1)

        # End-effector roll
        set_hand_roll_velocity(self.mcp1, int(roll_dir))

        # Grip movement
        open_close_hand(self.mcp3, int(grip_dir))

        # Poker movement
        # set_poker(self.poker_controller, poker_dir)

        # Flush stdout
        sys.stdout.flush()


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
        # self.poker_controller = Alfreds_Finger()
        ...
        
    


def main(args=None):

    rclpy.init(args=args)

    arm_driver = ArmDriver()

    rclpy.spin(arm_driver)

    # Destroy the node explicitly
    arm_driver.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
