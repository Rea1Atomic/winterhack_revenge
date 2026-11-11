'''
This file defined a node that:
    0. Intereact with hiwonder board
    1. Subscribe and eval the /cmd_vel
    2. Publish the /imu
'''
import rclpy

from math import pi, radians
from time import sleep
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from hiwonder_driver.hiwonder_sdk import Board

class HiwonderBoard(Node):
    def __init__(self):
        super().__init__("hiwonder_driver")

        # Parameters
        self.declare_parameter("board_device", "/dev/ttyACM0")

        # Initialize the board with hiwonder_sdk
        self.get_logger().info("Initialize driver with device: "
                             + str(self.get_parameter("board_device").value))

        self.board = Board(str(self.get_parameter("board_device").value))
        self.board.enable_reception()

        # imu calibrate
        while self.board.get_imu() == None:
            sleep(0.1)

        sleep(0.1)

        self.get_logger().info("Calibrating IMU...")

        imu_data_inc = [0, 0, 0, 0, 0, 0]
        imu_data_num = 0

        # get 30 sets of imu data in 30*0.1=3 seconds.
        # calculate the avg of them as the calibrated offset.
        for i in range(30):
            _data = self.board.get_imu()

            if _data == None:
                self.get_logger().warn("SDK returned None IMU value.")
            else:
                for i in range(len(_data)):
                    imu_data_inc[i] += _data[i]
                imu_data_num += 1

            sleep(0.1)

        self.imu_data_offset = [i / imu_data_num for i in imu_data_inc]
        self.get_logger().info(
                f"\nIMU calibrated with offset:\n{self.imu_data_offset}\n")

        # Subscriber of /cmd_vel. Callback func will set the motor speed
        # with command.
        self.cmd_subscriber = self.create_subscription(
                Twist,
                "cmd_vel",
                self.cmd_callback,
                10)

        # Publisher of imu data
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.imu_timer = self.create_timer(0.05, self.imu_timer_callback)

        # Bus servo init
        self.board.bus_servo_set_position(1, [[1, 500], [2, 500], [3, 312], [4, 125], [5, 500]])
        
        # Buzzer
        self.board.set_buzzer(4000, 0.5, 1, 1)

        self.get_logger().info("Board initialize success!")

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
        WIDTH = 0.1410      # Distance between left and right
        LENGTH = 0.1368     # Distance between front and back
        DIAMETER = 0.065    # Wheel diameter

        rps_l = - (msg.linear.x - (msg.angular.z * (WIDTH+LENGTH))/2) / (pi * DIAMETER)
        rps_r = (msg.linear.x + (msg.angular.z * (WIDTH+LENGTH))/2) / (pi * DIAMETER)

        self.board.set_motor_speed(
                [[1, rps_l], [2, rps_l], [3, rps_r], [4, rps_r]]
                )

        self.get_logger().debug(
                f'\nraw command:\t{msg.linear.x:.2f} m/s,'
                f'\t{msg.angular.z:.2f} rad/s\n'
                f'motor operation:\t{rps_l} rps,\t{rps_r} rps'
                )

    def imu_timer_callback(self):
        imu_raw_data = None;
        while imu_raw_data is None:
            imu_raw_data = self.board.get_imu()

        imu_raw_data_ros = Imu();
        imu_raw_data_ros.header.frame_id = "base_link"
        imu_raw_data_ros.header.stamp = self.get_clock().now().to_msg()
        imu_raw_data_ros.linear_acceleration.x = imu_raw_data[1] * 9.80665
        imu_raw_data_ros.linear_acceleration.y = -imu_raw_data[0] * 9.80665
        imu_raw_data_ros.linear_acceleration.z = imu_raw_data[2] * 9.80665
        imu_raw_data_ros.linear_acceleration_covariance = [0.0] * 9
        imu_raw_data_ros.angular_velocity.x = radians(imu_raw_data[4])
        imu_raw_data_ros.angular_velocity.y = radians(-imu_raw_data[3])
        imu_raw_data_ros.angular_velocity.z = radians(imu_raw_data[5])
        imu_raw_data_ros.angular_velocity_covariance = [0.0] * 9

        imu_raw_data_ros.orientation_covariance[0] = -1

        self.imu_publisher.publish(imu_raw_data_ros)

def main(args=None):
    rclpy.init(args=args)

    hc_node = HiwonderBoard()

    try:
        rclpy.spin(hc_node)
    finally:
        hc_node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        hc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
