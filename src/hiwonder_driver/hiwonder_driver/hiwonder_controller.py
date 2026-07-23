'''
This file defined a node that:
    1. Subscribe and eval the /cmd_vel
'''
import rclpy

from math import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hiwonder_driver.hiwonder_sdk import Board

class hiwonder_controller(Node):
    def __init__(self):
        super().__init__("hiwonder_controller")

        # Parameters
        self.declare_parameter("board_device", "/dev/ttyACM0")

        # Initialize the board with hiwonder_sdk
        self.get_logger().info("Initialize controller with device: "
                             + self.get_parameter("board_device").value
                             )
        self.board = Board(self.get_parameter("board_device").value)
        self.board.enable_reception()

        self.cmd_subscription = self.create_subscription(
                Twist,
                "cmd_vel",
                self.cmd_callback,
                10
                )

    def cmd_callback(self, msg):
        '''
        Motor Number:    ↑
                       1 | 3
                       2 | 4

        For 1/2 (Left):
            Negative for go ahead 
        For 3/4 (Right):
            Positive for go ahead
        '''
        WIDTH = 0.1410  # Distance between left and right
        LENGTH = 0.1368 # Distance between front and back
        DIAMETER = 0.065    # Wheel diameter

        rps_l = - (msg.linear.x - (msg.angular.z * (WIDTH+LENGTH))/2) / (pi * DIAMETER)
        rps_r = (msg.linear.x + (msg.angular.z * (WIDTH+LENGTH))/2) / (pi * DIAMETER) 

        self.board.set_motor_speed(
                [[1, rps_l], [2, rps_l], [3, rps_r], [4, rps_r]]
                )

        self.get_logger().info(
                f'raw command:\t{msg.linear.x} m/s,\t{msg.angular.z:.2f} rad/s\n'
                f'motor operation:\t{rps_l} rps,\t{rps_r} rps'
                )

        


def main(args=None):
    rclpy.init(args=args)

    hc_node = hiwonder_controller()

    try:
        rclpy.spin(hc_node)
    finally:
        hc_node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        hc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
