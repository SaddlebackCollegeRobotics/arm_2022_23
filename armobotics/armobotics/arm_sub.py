import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .lib.arm_controller import *

class Arm_IK_Sub(Node):

    def __init__(self):
        super().__init__('arm_ik_sub')
        self.subscription = self.create_subscription(
            String,
            'arm/ik_request',
            self.listener_callback,
            10)
        self.subscription