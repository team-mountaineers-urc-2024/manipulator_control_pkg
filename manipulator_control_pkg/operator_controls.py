import rclpy
import time
from sensor_msgs.msg import Joy
from robot_interfaces.msg import CanCommand, Status, ManipulatorControl
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8MultiArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from math import floor

'''
Linear rail

Shoulder
Elbow

Wrist roll
Wrist pitch

gripper
solenoid
screw driver

id number 
claw 33
screw dirver 34
solenoid 35
pump 36
linear actuator 37

order:
solenoid: 1 - forward, 0 - retracts 
direction for claw device


[device, id, direction, duty cycle]
33-37, claw needs to id

2nd only for solenoid


there may be some delay as I can only send to one motor at a time 
'''

class OperatorControlNode(Node):

    def __init__(self):
        
        self.restart_safety_count: int = 0

        super().__init__('operator_arm_controls_node')

        self.declare_parameters(
            namespace='',
            parameters=[

                ('manipulator_control_topic', Parameter.Type.STRING),
                ('manipulator_joystick_topic', Parameter.Type.STRING),
                ('pico_topic_name', Parameter.Type.STRING),

                ('debugging', Parameter.Type.INTEGER),

                ('max_dps', Parameter.Type.INTEGER),

                ('required_presses_for_restart_btn', Parameter.Type.INTEGER),
                ('joy_msg_indexes.restart_btn_index', Parameter.Type.INTEGER),

                ('joy_msg_indexes.linear_rail_left_btn_index', Parameter.Type.INTEGER),
                ('joy_msg_indexes.linear_rail_right_btn_index', Parameter.Type.INTEGER),

                ('joy_msg_indexes.shoulder_control_axes_index', Parameter.Type.INTEGER),
                ('joy_msg_indexes.elbow_control_axes_index', Parameter.Type.INTEGER),

                ('joy_msg_indexes.wrist_speed_control_axes_index', Parameter.Type.INTEGER),
                ('joy_msg_indexes.wrist_roll_control_axes_index', Parameter.Type.INTEGER),
                ('joy_msg_indexes.wrist_pitch_control_axes_index', Parameter.Type.INTEGER),

                ('joy_msg_indexes.gripper_open_btn_index', Parameter.Type.INTEGER),
                ('joy_msg_indexes.gripper_close_btn_index', Parameter.Type.INTEGER),

                ('joy_msg_indexes.screwdriver_control_axes_index', Parameter.Type.INTEGER),
                ('joy_msg_indexes.solenoid_control_btn_index', Parameter.Type.INTEGER),

                ('pico_internal.max_pwm_duty_cycle', Parameter.Type.INTEGER),
                ('pico_internal.dc_motor_ids.gripper', Parameter.Type.INTEGER),
                ('pico_internal.dc_motor_ids.screw_driver', Parameter.Type.INTEGER),
                ('pico_internal.dc_motor_ids.solenoid', Parameter.Type.INTEGER),

                ('pico_pub_rate', 10),

                ('do_heartbeat', False),
                ('heartbeat_topic', 'reciever_communication'),
                ('message_timeout', 0.5)

            ]
        )

        # Publisher for Arm commands
        self.get_logger().info(f'\033[34mCreating Arm Command Publisher\033[0m')
        self.control_pub = self.create_publisher(
            msg_type=ManipulatorControl,
            topic= self.get_parameter('manipulator_control_topic').get_parameter_value().string_value,
            qos_profile=10
            )
        self.get_logger().info(f'\033[32mArm Command Publisher Created\033[0m')

        # Publisher for pico commands
        self.get_logger().info(f'\033[34mCreating Pico Command Publisher\033[0m')
        self.pico_publisher = self.create_publisher(
            msg_type=UInt8MultiArray,
            topic= self.get_parameter('pico_topic_name').get_parameter_value().string_value,
            qos_profile=10
            )
        self.get_logger().info(f'\033[32mPico Command Publisher Created\033[0m')

        # Subscribe to Joystick values
        self.get_logger().info(f'\033[34mCreating Arm Joystick Subscriber\033[0m')
        self.joy_sub = self.create_subscription(
            msg_type= Joy,
            topic= self.get_parameter('manipulator_joystick_topic').get_parameter_value().string_value,
            callback= self.send_manipulator_control,
            qos_profile= qos_profile_sensor_data
            )
        self.get_logger().info(f'\033[32mArm Joystick Subscriber Created\033[0m')
    
        # Timer to publish pico messages at a regular rate
        self.get_logger().info(f'\033[34mCreating Pico Command Timer\033[0m')
        self.pico_timer = self.create_timer(
            1/(self.get_parameter('pico_pub_rate').get_parameter_value().integer_value),
            self.timer_callback
            )
        self.get_logger().info(f'\033[32mPico Command Timer Created\033[0m')
        
        # Subscribe to the heartbeat
        self.get_logger().info(f'\033[34mCreating Heartbeat Subscriber\033[0m')
        self.heartbeat_sub = self.create_subscription(
            msg_type= Status,
            topic=self.get_parameter('heartbeat_topic').get_parameter_value().string_value,
            callback= self.heartbeat_callback,
            qos_profile=10
        )
        self.get_logger().info(f'\033[32mHeartbeat Subscriber Created\033[0m')

        # Default Pico messages to eliminate compile errors
        self.gripper_msg            = UInt8MultiArray()
        self.gripper_msg.data       = [self.get_parameter('pico_internal.dc_motor_ids.gripper').get_parameter_value().integer_value, 1, 0, 0]
        self.screwdriver_msg        = UInt8MultiArray()
        self.screwdriver_msg.data   = [self.get_parameter('pico_internal.dc_motor_ids.screw_driver').get_parameter_value().integer_value, 1, 0, 0]
        self.solenoid_msg           = UInt8MultiArray()
        self.solenoid_msg.data      = [self.get_parameter('pico_internal.dc_motor_ids.solenoid').get_parameter_value().integer_value, 1, 0, 0]

        # Values to make sure arm doesn't kill itself on init (Joy sometimes defaults to max)
        self.get_logger().info(f'\033[34mShoulder and Elbow speed set to zero\033[0m')
        self.shoulder_ok    = False
        self.elbow_ok       = False
        self.last_msg       = 0.0

        # If the heartbeat is up, 
        if (self.get_parameter('do_heartbeat').get_parameter_value().bool_value):
            self.zero = True
        else:
            self.zero = False

    # Heartbeat Callback
    def heartbeat_callback(self, status_msg):
        
        if (not self.zero and status_msg.isup):
            self.get_logger().info(f'\033[32mArm Enabled by Heartbeat\033[0m')

        elif (self.zero and not status_msg.isup):
            self.get_logger().info(f'\033[31mArm Disabled by Heartbeat\033[0m')

        self.zero = status_msg.isup


    def send_manipulator_control(self, joy_msg):

        def get_param_int(name: str):
            return self.get_parameter(name).get_parameter_value().integer_value
        
    #grabbing annoying ros object orientation bs, it is nice tho thx ros ppl
        max_dps                             = get_param_int('max_dps')
        required_presses_for_restart_btn    = get_param_int('required_presses_for_restart_btn')
        restart_btn_index                   = get_param_int('joy_msg_indexes.restart_btn_index')
        linear_rail_left_btn_index          = get_param_int('joy_msg_indexes.linear_rail_left_btn_index')
        linear_rail_right_btn_index         = get_param_int('joy_msg_indexes.linear_rail_right_btn_index')
        shoulder_control_axes_index         = get_param_int('joy_msg_indexes.shoulder_control_axes_index')
        elbow_control_axes_index            = get_param_int('joy_msg_indexes.elbow_control_axes_index')
        wrist_speed_control_axes_index      = get_param_int('joy_msg_indexes.wrist_speed_control_axes_index')
        wrist_roll_control_axes_index       = get_param_int('joy_msg_indexes.wrist_roll_control_axes_index')
        wrist_pitch_control_axes_index      = get_param_int('joy_msg_indexes.wrist_pitch_control_axes_index')
        gripper_open_btn_index              = get_param_int('joy_msg_indexes.gripper_open_btn_index')
        gripper_close_btn_index             = get_param_int('joy_msg_indexes.gripper_close_btn_index')
        screwdriver_control_axes_index      = get_param_int('joy_msg_indexes.screwdriver_control_axes_index')
        solenoid_control_btn_index          = get_param_int('joy_msg_indexes.solenoid_control_btn_index')

        max_duty_cycle  = get_param_int('pico_internal.max_pwm_duty_cycle')
        gripper_id      = get_param_int('pico_internal.dc_motor_ids.gripper')
        screw_driver_id = get_param_int('pico_internal.dc_motor_ids.screw_driver')
        solenoid_id     = get_param_int('pico_internal.dc_motor_ids.solenoid')

        myactuator_manipulator_control_msg = ManipulatorControl()

        gripper_close:  int = 1
        gripper_open:   int = 0

        spin_cw:    int = 1
        spin_ccw:   int = 0

        push_forward:   int = 1 
        retract:        int = 0 

        def clamp(val) -> int:
            return max(-1,min(val,1))

        def normalize_joystick_axes_vals(value: float):
            return clamp(-1 * value)
        
        def normalize_bumper_axes_vals(value: float):
            return clamp(1 - ((value + 1)/2))

        # ------------
        # PICO PORTION
        # ------------

        def make_pico_msg(device_id, direction, duty_cycle):
            pico_msg = UInt8MultiArray()
            pico_msg.data = [device_id, 1, direction, duty_cycle]
            return pico_msg

        # Gripper

        # If both buttons are the same do nothing
        if (joy_msg.buttons[gripper_open_btn_index] == joy_msg.buttons[gripper_close_btn_index]):
            self.gripper_msg = make_pico_msg(gripper_id, gripper_open, 0)

        # If open button, open
        elif(joy_msg.buttons[gripper_open_btn_index] == 1):
            self.gripper_msg = make_pico_msg(gripper_id, gripper_open, max_duty_cycle)

        # If close button, close
        elif(joy_msg.buttons[gripper_close_btn_index] == 1):
            self.gripper_msg = make_pico_msg(gripper_id, gripper_close, max_duty_cycle)

        # Base Case
        else:
            self.get_logger().error("Gripper Somehow Hit Base Case, zeroing value")
            self.gripper_msg = make_pico_msg(gripper_id, gripper_close, 0)

        # Screwdriver
        self.screwdriver_msg = make_pico_msg(screw_driver_id, spin_cw, int(floor(normalize_bumper_axes_vals(joy_msg.axes[screwdriver_control_axes_index]) * max_duty_cycle)))
        
        # Solenoid -> If pressed out if not in
        if (joy_msg.buttons[solenoid_control_btn_index] == 1):
            self.solenoid_msg = make_pico_msg(solenoid_id, push_forward, max_duty_cycle)
        else:
            self.solenoid_msg = make_pico_msg(solenoid_id, retract, max_duty_cycle)

        # ------------------
        # MYACTUATOR PORTION
        # ------------------
        if(joy_msg.buttons[restart_btn_index] == 1):
            self.restart_safety_count += 1 

        if (self.restart_safety_count >= required_presses_for_restart_btn):
            self.restart_safety_count = 0
            myactuator_manipulator_control_msg.restart_motors = True


        # Check if the controllers have disconnected

        # If this is the first message, save the time
        if (self.last_msg == 0):
            self.last_msg = time.time()
        
        # Get and compare the current time
        cur_msg = time.time()

        if (cur_msg - self.last_msg) > self.get_parameter('message_timeout').get_parameter_value().double_value:
            self.get_logger().info(f'\033[31mArm Message Timeout Reached - Controller may have died\033[0m')
            self.get_logger().info(f'\033[93mShoulder and Elbow speed set to zero\033[0m')
            self.shoulder_ok = False
            self.elbow_ok = False

        # Overwrite joy_values if it isn't safe to move yet
        if not self.shoulder_ok:
            # If values are stuck at max, set them to zero
            if joy_msg.axes[shoulder_control_axes_index] == 1.0:

                joy_msg.axes[shoulder_control_axes_index] = 0.0
            else:
                self.get_logger().info(f'\033[32mShoulder Joystick Control Enabled\033[0m')
                self.shoulder_ok = True

        if not self.elbow_ok:
            # If values are stuck at max, set them to zero
            if joy_msg.axes[elbow_control_axes_index] == 1.0:

                joy_msg.axes[elbow_control_axes_index] = 0.0
            else:
                self.get_logger().info(f'\033[32mElbow Joystick Control Enabled\033[0m')
                self.elbow_ok = True
        
        # Linear Rail 
        myactuator_manipulator_control_msg.linear_rail_dps = floor((joy_msg.buttons[linear_rail_left_btn_index] - joy_msg.buttons[linear_rail_right_btn_index]) * 1000)
        # Shoulder
        myactuator_manipulator_control_msg.shoulder_dps = floor(normalize_joystick_axes_vals(joy_msg.axes[shoulder_control_axes_index]) * max_dps * 0.5)
        # Elbow 
        myactuator_manipulator_control_msg.elbow_dps = floor(normalize_joystick_axes_vals(joy_msg.axes[elbow_control_axes_index]) * max_dps * 0.3)
        # Speed of Wrist 
        wrist_speed_coefficient = normalize_bumper_axes_vals(joy_msg.axes[wrist_speed_control_axes_index])
        # Wrist Roll 
        myactuator_manipulator_control_msg.wrist_roll_dps = int(joy_msg.axes[wrist_roll_control_axes_index] * wrist_speed_coefficient * max_dps * 3)
        # Wrist Pitch
        myactuator_manipulator_control_msg.wrist_pitch_dps = int(joy_msg.axes[wrist_pitch_control_axes_index] * wrist_speed_coefficient * max_dps * -1)

        if self.get_parameter('debugging').get_parameter_value().integer_value:
            self.get_logger().info(f"Shoulder DPS: {myactuator_manipulator_control_msg.shoulder_dps}")
            self.get_logger().info(f"Elbow DPS:    {myactuator_manipulator_control_msg.elbow_dps}")
            self.get_logger().info(f"Gripper:      {self.gripper_msg.data}")

        # Publish zero if we want to stop
        if (not self.zero):
            self.control_pub.publish(myactuator_manipulator_control_msg)
        else:
            zero = ManipulatorControl()
            self.control_pub.publish(zero)

        # Update the time the last message was sent
        self.last_msg = cur_msg


    def timer_callback(self):
        for pico_msg in [self.gripper_msg, self.screwdriver_msg, self.solenoid_msg]:
            self.pico_publisher.publish(pico_msg)
    

def main(args=None):
    rclpy.init(args=args)
    manipulator_node = OperatorControlNode()
    rclpy.spin(manipulator_node)
    manipulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
