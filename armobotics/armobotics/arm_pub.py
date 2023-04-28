import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Arm_IK_Pub(Node):

    def __init__(self):
        super().__init__('arm_ik_pub')
        self.publisher_ = self.create_publisher(String, 'arm/ik_request', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        