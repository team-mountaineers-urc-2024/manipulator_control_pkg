
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

class ManipulatorControlNodeUart(Node):

    def __init__(self):

        super().__init__('arm_uart_node')

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
    
        self.wrist_roll     = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.wrist_roll').get_parameter_value().integer_value)
        self.linear_rail    = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.linear_rail').get_parameter_value().integer_value)
        self.shoulder       = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.shoulder').get_parameter_value().integer_value)
        self.wrist_pitch    = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.wrist_pitch').get_parameter_value().integer_value)
        self.elbow          = MyActuatorMotor(self.get_parameter('can_ids.manipulator_motors.elbow').get_parameter_value().integer_value)
    
        self.get_logger().info(f'\033[34mCreating UART Publisher\033[0m')
        self.uart_command_publisher = self.create_publisher(
            msg_type    = UInt8MultiArray,
            topic       = self.get_parameter('uart_topic_name').get_parameter_value().string_value,
            qos_profile = 10
            )
        self.get_logger().info(f'\033[32mUART Publisher Created\033[0m')

        self.get_logger().info(f'\033[34mCreating Arm Command Subscriber\033[0m')
        self.control_sub = self.create_subscription(
            msg_type    = ManipulatorControl,
            topic       = self.get_parameter('manipulator_control_topic').get_parameter_value().string_value,
            callback    = self.send_arm_command,
            qos_profile = 1
            )
        self.get_logger().info(f'\033[32mArm Command Subscriber Created\033[0m')
        
        self.get_logger().info(f'\033[34mInitializing Arm UART Motors\033[0m')

        # Uart Port for the Wrist Roll
        self.wrist_roll_port = serial.Serial(
            port        =  "/dev/ttyUSB0",
            baudrate    = self.get_parameter('uart_baudrate').get_parameter_value().integer_value,
            timeout     = 1
            )
        self.get_logger().info(f'\033[32mWrist Roll: Initialized\033[0m')
        
        # Uart Port for the Linear Rail
        self.linear_rail_port = serial.Serial(
            port        = "/dev/ttyUSB1",
            baudrate    = self.get_parameter('uart_baudrate').get_parameter_value().integer_value,
            timeout     = 1
            )
        self.get_logger().info(f'\033[32mShoulder Linear Rail: Initialized\033[0m')
        
        # # Uart Port for the Shoulder
        # self.shoulder_port = serial.Serial(
        #     port        =  "/dev/ttyUSB2",
        #     baudrate    = self.get_parameter('uart_baudrate').get_parameter_value().integer_value,
        #     timeout     = 1
        #     )
        # self.get_logger().info(f'\033[32mShoulder: Initialized\033[0m')
        
        # # Uart Port for the Wrist Pitch
        # self.wrist_pitch_port = serial.Serial(
        #     port        = "/dev/ttyUSB3",
        #     baudrate    = self.get_parameter('uart_baudrate').get_parameter_value().integer_value,
        #     timeout     = 1
        #     )
        # self.get_logger().info(f'\033[32mWrist Pitch: Initialized\033[0m')
        
        # # Uart Port for the Elbow
        # self.elbow_port = serial.Serial(
        #     port        = "/dev/ttyUSB4",
        #     baudrate    = self.get_parameter('uart_baudrate').get_parameter_value().integer_value,
        #     timeout     = 1
        #     )
        # self.get_logger().info(f'\033[32mElbow: Initialized\033[0m')

    # When an arm command is recieved run this
    def send_arm_command(self, manipulator_control_msg):

        # If the command was to restart the motors, restart them
        if(manipulator_control_msg.restart_motors):

            # Reset the Wrist Roll Motor
            self.send_uart_message(
                self.linear_rail_port, 
                self.get_parameter('can_ids.manipulator_motors.linear_rail').get_parameter_value().integer_value, 
                self.linear_rail.System_reset_command()
            )
            self.get_logger().info(f'\033[31mLinear Rail Motor Reset\033[0m')
            
            # Reset the Linear Rail Motor
            self.send_uart_message(
                self.wrist_roll_port, 
                self.get_parameter('can_ids.manipulator_motors.wrist_roll').get_parameter_value().integer_value, 
                self.wrist_roll.System_reset_command()
            )
            self.get_logger().info(f'\033[31mWrist Roll Motor Reset\033[0m')

            # # Reset the Shoulder Motor
            # self.send_uart_message(
            #     self.shoulder_port, 
            #     self.get_parameter('can_ids.manipulator_motors.shoulder').get_parameter_value().integer_value, 
            #     self.shoulder.System_reset_command()
            # )
            # self.get_logger().info(f'\033[31mShoulder Motor Reset\033[0m')
            
            # # Reset the Wrist Pitch Motor
            # self.send_uart_message(
            #     self.wrist_pitch_port, 
            #     self.get_parameter('can_ids.manipulator_motors.wrist_pitch').get_parameter_value().integer_value, 
            #     self.wrist_pitch.System_reset_command()
            # )
            # self.get_logger().info(f'\033[31mWrist Pitch Motor Reset\033[0m')


            # # Reset the Elbow Motor
            # self.send_uart_message(
            #     self.elbow_port, 
            #     self.get_parameter('can_ids.manipulator_motors.elbow').get_parameter_value().integer_value, 
            #     self.elbow.System_reset_command()
            # )
            # self.get_logger().info(f'\033[31mElbow Motor Reset\033[0m')


        # If it is a normal command
        else:

            # Send the Wrist Roll Value
            self.send_uart_message(
                self.wrist_roll_port, 
                self.get_parameter('can_ids.manipulator_motors.wrist_roll').get_parameter_value().integer_value, 
                self.wrist_roll.Speed_Closed_loop_Control_Command(manipulator_control_msg.wrist_roll_dps)
            )

            # Send the Linear Rail Value
            self.send_uart_message(
                self.linear_rail_port, 
                self.get_parameter('can_ids.manipulator_motors.linear_rail').get_parameter_value().integer_value, 
                self.linear_rail.Speed_Closed_loop_Control_Command(manipulator_control_msg.linear_rail_dps)
            )

            # # Reset the Shoulder Motor
            # self.send_uart_message(
            #     self.shoulder_port, 
            #     self.get_parameter('can_ids.manipulator_motors.shoulder').get_parameter_value().integer_value, 
            #     self.shoulder.Speed_Closed_loop_Control_Command(manipulator_control_msg.shoulder_dps)
            # )
            
            # # Reset the Wrist Pitch Motor
            # self.send_uart_message(
            #     self.wrist_pitch_port, 
            #     self.get_parameter('can_ids.manipulator_motors.wrist_pitch').get_parameter_value().integer_value, 
            #     self.wrist_pitch.Speed_Closed_loop_Control_Command(manipulator_control_msg.wrist_pitch_dps)
            # )

            # # Reset the Elbow Motor
            # self.send_uart_message(
            #     self.elbow_port, 
            #     self.get_parameter('can_ids.manipulator_motors.elbow').get_parameter_value().integer_value, 
            #     self.elbow.Speed_Closed_loop_Control_Command(manipulator_control_msg.elbow_dps)
            # )

    def send_uart_message(self, port: serial.Serial, id: int, command: can.Message):
        command = command.data

        header = [0x3e, id - 0x140, 0x08]
        command = list(command)
        crc = computeCRC(bytes(header + command))          
        values = bytearray(header + command + [crc>>8, crc&0xFF])

        if self.get_parameter('debugging').get_parameter_value().integer_value:
            self.get_logger().info(f'sending uart: {values}')
        
        uintarray_msg = UInt8MultiArray()
        uintarray_msg.data = values

        # Debuging script
        if self.get_parameter('debugging').get_parameter_value().integer_value:
            self.get_logger().info(f'sending uart over ros: {uintarray_msg}')

        # Publish output for ros viewing
        self.uart_command_publisher.publish(uintarray_msg)

        # Write and read uart data
        port.write(values)
        if self.get_parameter('debugging').get_parameter_value().integer_value:
            read_data = port.read(13)
            self.get_logger().info(f'reply uart: {read_data}')

    
def main(args=None):
    rclpy.init(args=args)
    manipulator_node = ManipulatorControlNodeUart()
    rclpy.spin(manipulator_node)
    manipulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
