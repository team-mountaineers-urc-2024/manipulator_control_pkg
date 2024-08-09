
import math, rclpy, time, operator, can
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from robot_interfaces.msg import CanCommand, ManipulatorControl
from myactuator_lib import Motor as MyActuatorMotor
from rclpy.node import Node
from rclpy.parameter import Parameter, ParameterValue
import serial
from pymodbus.utilities import computeCRC
from std_msgs.msg import UInt8MultiArray

# linear rail 026


class ManipulatorControlNodeCan(Node):

    def __init__(self):

        super().__init__('arm_can_node')

        # Unused parameters are left in for legacy and future use if UART is necessary once again
        self.declare_parameters(
            namespace='',
            parameters=[
                ('can_ids.manipulator_motors.elbow', Parameter.Type.INTEGER),
                ('can_ids.manipulator_motors.shoulder', Parameter.Type.INTEGER),
                ('can_ids.manipulator_motors.linear_rail', Parameter.Type.INTEGER),
                ('can_ids.manipulator_motors.wrist_pitch', Parameter.Type.INTEGER),
                ('can_ids.manipulator_motors.wrist_roll', Parameter.Type.INTEGER),

                ('can_topic_name', Parameter.Type.STRING),
                ('uart_topic_name',Parameter.Type.STRING),
                ('manipulator_control_topic', Parameter.Type.STRING),
                ('max_dps', Parameter.Type.INTEGER),
                ('debugging', Parameter.Type.INTEGER),

                ('uart_path_wrist_roll', Parameter.Type.STRING),
                ('uart_path_linear_rail', Parameter.Type.STRING),
                ('uart_baudrate', 115200)
            ]
        )

        # Make the CAN Publisher
        self.get_logger().info(f'\033[34mCreating CAN Publisher\033[0m')
        self.can_command_publisher = self.create_publisher(
            msg_type= CanCommand,
            topic= self.get_parameter('can_topic_name').get_parameter_value().string_value,
            qos_profile= 10
            )
        self.get_logger().info(f'\033[32mCAN Publisher Created\033[0m')


        # Make the Arm Command Subscriber
        self.get_logger().info(f'\033[34mCreating Arm Command Subscriber\033[0m')
        self.control_sub = self.create_subscription(
            msg_type= ManipulatorControl,
            topic= self.get_parameter('manipulator_control_topic').get_parameter_value().string_value,
            callback= self.send_arm_command,
            qos_profile= 1
            )
        self.get_logger().info(f'\033[32mArm Command Subscriber Created\033[0m')
        
        # Initialize all the motors
        self.get_logger().info(f'\033[34mInitializing Arm CAN Motors\033[0m')

        self.shoulder = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.shoulder').get_parameter_value().integer_value)
        self.get_logger().info(f'\033[32mShoulder: Initialized\033[0m')

        self.elbow = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.elbow').get_parameter_value().integer_value)
        self.get_logger().info(f'\033[32mElbow: Initialized\033[0m')

        self.linear_rail = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.linear_rail').get_parameter_value().integer_value)
        self.get_logger().info(f'\033[32mShoulder Linear Rail: Initialized\033[0m')

        self.wrist_pitch = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.wrist_pitch').get_parameter_value().integer_value)
        self.get_logger().info(f'\033[32mWrist Pitch: Initialized\033[0m')

        self.wrist_roll = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.wrist_roll').get_parameter_value().integer_value)
        self.get_logger().info(f'\033[32mWrist Roll: Initialized\033[0m')

    def send_arm_command(self, manipulator_control_msg):

        # Match each motor with its current dps
        # Comment out these lines if motors blow up
        can_motor_mappings = {
            self.elbow          : manipulator_control_msg.elbow_dps,
            self.linear_rail    : manipulator_control_msg.linear_rail_dps,
            self.wrist_pitch    : manipulator_control_msg.wrist_pitch_dps,
            self.wrist_roll     : manipulator_control_msg.wrist_roll_dps,
            self.shoulder       : manipulator_control_msg.shoulder_dps,
        }

        # If we are supposed to be restarting the motors
        if(manipulator_control_msg.restart_motors):
            self.get_logger().info(f'\033[31mMotors asked to Reset\033[0m')

            # Loop through the list of them and send each a restart command
            for can_motor, dps in can_motor_mappings.items():
                self.send_can_message(can_motor.System_reset_command())

        # Otherwise pass the speed on to the motor
        else:

            # Loop through the list of motors and send them the correct speed
            for can_motor, dps in can_motor_mappings.items():
                self.send_can_message(can_motor.Speed_Closed_loop_Control_Command(dps))

    # Parse and publish the commands to the can topic for motor writing
    def send_can_message(self, can_command: can.Message):

        can_outgoing_ros_message = CanCommand()
        can_outgoing_ros_message.arbitration_id = can_command.arbitration_id
        can_outgoing_ros_message.is_extended_id = can_command.is_extended_id
        can_outgoing_ros_message.byte_0 = can_command.data[0]
        can_outgoing_ros_message.byte_1 = can_command.data[1]
        can_outgoing_ros_message.byte_2 = can_command.data[2]
        can_outgoing_ros_message.byte_3 = can_command.data[3]
        can_outgoing_ros_message.byte_4 = can_command.data[4]
        can_outgoing_ros_message.byte_5 = can_command.data[5]
        can_outgoing_ros_message.byte_6 = can_command.data[6]
        can_outgoing_ros_message.byte_7 = can_command.data[7]

        if self.get_parameter('debugging').get_parameter_value().integer_value:
            self.get_logger().info(f'sending can: {can_outgoing_ros_message}')

        self.can_command_publisher.publish(can_outgoing_ros_message)

   
def main(args=None):
    rclpy.init(args=args)
    manipulator_node = ManipulatorControlNodeCan()
    rclpy.spin(manipulator_node)
    manipulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
