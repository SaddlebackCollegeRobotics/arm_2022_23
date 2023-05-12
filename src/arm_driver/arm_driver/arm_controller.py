
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
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.axis_deadzone = 0.1

        # Initialize gamepad input listener
        gamepad_input.setConfigFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../share/arm_driver/gamepads.config'))
        gamepad_input.run_event_loop()


    # Publish button and axis input to ROS2 topic
    def timer_callback(self):
        
        msg = Float32MultiArray()
        gamepad = gamepad_input.getGamepad(0)

        (l2, r2) = gamepad_input.getTriggers(gamepad, self.axis_deadzone)
        (ls_x, ls_y) = gamepad_input.getLeftStick(gamepad, self.axis_deadzone)
        (rs_x, rs_y) = gamepad_input.getRightStick(gamepad, self.axis_deadzone)
        (hat_x, hat_y) = gamepad_input.getHat(gamepad)

        msg.data = [float(l2), float(r2), float(ls_x), float(ls_y), float(rs_x), float(rs_y), float(hat_x), float(hat_y)]

        for i in range(0, 11):
           msg.data.append(1.0) if gamepad_input.getButtonValue(gamepad, i) else msg.data.append(0.0)

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
