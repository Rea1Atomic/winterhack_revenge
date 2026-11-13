'''
This file defined a node that:
    0. Intereact with hiwonder board
    1. Subscribe and eval the /cmd_vel
    2. Publish the /imu
    3. Provide arm action server
    4. Provide gripper action server
    5. Publish the /joint_states
'''
import rclpy

from math import pi, radians
from time import sleep

from rclpy import action
from rclpy import duration
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.clock import Clock, ClockType
from rclpy.task import Future

from rclpy.qos import Duration
from rosidl_parser.definition import Action
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

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
        self.get_logger().info(f"IMU calibrated with offset:\n{self.imu_data_offset}")

        # Subscriber of /cmd_vel. Callback func will set the motor speed
        # with command.
        self.cmd_subscriber = self.create_subscription(
                Twist,
                "cmd_vel",
                self.cmd_callback,
                10)

        # Publisher of /joint_states
        self.joint_states_publisher = self.create_publisher(
                JointState,
                "/joint_states",
                10
        )
        self.joint_states_timer = self.create_timer(
                0.1,
                self.joint_states_timer_callback
        )

        # Arm action server
        self._arm_goal = None
        self._arm_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_trajectory_controller/follow_joint_trajectory',
            self.arm_action_callback,
            goal_callback=self.arm_goal_callback,
            handle_accepted_callback=self.arm_goal_accepted,
        )
        # Gripper action server
        self._gripper_goal = None
        self._gripper_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_trajectory_controller/follow_joint_trajectory',
            self.gripper_action_callback,
            goal_callback=self.gripper_goal_callback,
            handle_accepted_callback=self.gripper_goal_accepted,
        )


        # Publisher of imu data
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.imu_timer = self.create_timer(0.05, self.imu_timer_callback)

        # Bus servo init
        self.board.bus_servo_set_position(5, [[1, 500], [2, 500], [3, 500], [4, 500], [5, 500], [6, 200]])
        
        # Buzzer
        self.board.set_buzzer(4000, 0.5, 1, 1)
        self.get_logger().info("Board initialize success!")

    def cmd_callback(self, msg: Twist):
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

    def joint_states_timer_callback(self):
        state_sdk_data = []
        for i in range(1, 7):
            _data = self.board.bus_servo_read_position(i)
            if _data is not None:
                state_sdk_data.append(_data[0])
            else:
                state_sdk_data.append(500)
                self.get_logger().warn("bus_servo_read_position returned None")

        state_ros_data = JointState()
        state_ros_data.header.frame_id = ''
        state_ros_data.header.stamp = self.get_clock().now().to_msg()
        state_ros_data.velocity = []
        state_ros_data.effort = []
        state_ros_data.name = []

        for i in range(0, 5):
            state_ros_data.name.append('joint' + str(i+1))
            state_ros_data.position.append((state_sdk_data[i]-500) * pi / 750)

        state_ros_data.name.append('gripper_base_joint')
        state_ros_data.position.append(-90/180*pi - (state_sdk_data[5]-500) * pi / 750)

        self.joint_states_publisher.publish(state_ros_data)

    def arm_goal_callback(self, _goal_request):
        if self._arm_goal is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def gripper_goal_callback(self, _goal_request):
        if self._gripper_goal is not None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT


    def arm_goal_accepted(self, action_handle: ServerGoalHandle):
        # With the single-threaded executor, execution starts after this callback returns.
        action_handle.execute()
        self._arm_goal = action_handle

    def gripper_goal_accepted(self, action_handle: ServerGoalHandle):
        # With the single-threaded executor, execution starts after this callback returns.
        action_handle.execute()
        self._gripper_goal = action_handle


    async def _wait_for_arm_segment(self, seconds):
        elapsed = Future(executor=self.executor)

        def on_timeout():
            timer.cancel()
            elapsed.set_result(None)

        timer = self.create_timer(seconds, on_timeout, clock=self._arm_clock)
        try:
            await elapsed
        finally:
            self.destroy_timer(timer)

    async def _wait_for_gripper_segment(self, seconds):
        elapsed = Future(executor=self.executor)

        def on_timeout():
            timer.cancel()
            elapsed.set_result(None)

        timer = self.create_timer(seconds, on_timeout, clock=self._gripper_clock)
        try:
            await elapsed
        finally:
            self.destroy_timer(timer)


    async def arm_action_callback(self, action_handle: ServerGoalHandle):
        try:
            self.get_logger().info('Received a new arm goal. Executing...')
            traj: JointTrajectory = action_handle.request.trajectory
            ids = [int(name[-1]) for name in traj.joint_names]
            last_point_time_ns = 0

            for point in traj.points:
                stamp = point.time_from_start
                point_time_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                duration_s = (point_time_ns - last_point_time_ns) / 1_000_000_000

                cmd = [
                    [servo_id, int(500 + point.positions[i] * 750 / pi)]
                    for i, servo_id in enumerate(ids)
                ]
                self.board.bus_servo_set_position(duration_s, cmd)
                if duration_s > 0:
                    await self._wait_for_arm_segment(duration_s)
                last_point_time_ns = point_time_ns

            action_handle.succeed()
            return FollowJointTrajectory.Result()
        finally:
            self._arm_goal = None

    async def gripper_action_callback(self, action_handle: ServerGoalHandle):
        try:
            self.get_logger().info('Received a new gripper goal. Executing...')
            traj: JointTrajectory = action_handle.request.trajectory
            last_point_time_ns = 0

            for point in traj.points:
                stamp = point.time_from_start
                point_time_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                duration_s = (point_time_ns - last_point_time_ns) / 1_000_000_000

                self.board.bus_servo_set_position(duration_s, [[6, int(500 + (-90/180*pi - point.positions[0]) * 750 / pi)]])
                if duration_s > 0:
                    await self._wait_for_gripper_segment(duration_s)
                last_point_time_ns = point_time_ns

            action_handle.succeed()
            return FollowJointTrajectory.Result()
        finally:
            self._gripper_goal = None


    def imu_timer_callback(self):
        imu_raw_data = self.board.get_imu()

        if imu_raw_data is None:
            return

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
