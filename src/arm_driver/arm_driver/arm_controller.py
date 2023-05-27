# Author: Cameron Rosenthal - @Supernova1114

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# General imports
from . import gamepad_input
import os


class Arm_Controller(Node):

    def __init__(self):

        # Give the node a name.
        super().__init__('arm_controller')

        # Specify data type and topic name. Specify queue size (limit amount of queued messages)
        self.publisher_ = self.create_publisher(Float32MultiArray, '/arm/control_instruction', 10)

        # Create a timer that will call the 'timer_callback' function every timer_period second.
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.axis_deadzone = 0.1

        # Initialize gamepad input listener
        gamepad_input.setConfigFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../share/arm_driver/gamepads.config'))
        gamepad_input.run_event_loop()


    # Publish button and axis input to ROS2 topic
    def timer_callback(self):
        
        msg = Float32MultiArray()
        gamepad = gamepad_input.getGamepad(1)


        # (l2, r2) = gamepad_input.getTriggers(gamepad, self.axis_deadzone)
        (ls_x, ls_y) = gamepad_input.getLeftStick(gamepad, self.axis_deadzone)
        (rs_x, rs_y) = gamepad_input.getRightStick(gamepad, self.axis_deadzone)
        (hat_x, hat_y) = gamepad_input.getHat(gamepad)

        bicep_dir = rs_y
        forearm_dir = ls_y

        turret_dir = ls_x

        pitch_dir = hat_y
        roll_dir = hat_x

        # X close, O open
        grip_dir = 1.0 if gamepad_input.getButtonValue(gamepad, 2) else -1.0 if gamepad_input.getButtonValue(gamepad, 3) else 0

        # Triangle out, Square in
        poker_dir = 1.0 if gamepad_input.getButtonValue(gamepad, 0) else -1.0 if gamepad_input.getButtonValue(gamepad, 1) else 0

        # Menu button
        safety_trigger = 1 if gamepad_input.getButtonValue(gamepad, 5) else 0

        # Pack ROS2 message
        msg.data = [float(bicep_dir), float(forearm_dir), float(-turret_dir), float(-pitch_dir), float(roll_dir), float(-grip_dir), float(poker_dir), float(safety_trigger)]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    arm_controller = Arm_Controller()

    rclpy.spin(arm_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
