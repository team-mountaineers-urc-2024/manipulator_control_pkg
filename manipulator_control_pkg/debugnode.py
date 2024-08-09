import rclpy
import time
from sensor_msgs.msg import Joy
from robot_interfaces.msg import ManipulatorControl
from rclpy.node import Node
import csv 
from datetime import datetime

class OperatorControlNode(Node):

    def __init__(self):
        
        self.restart_safety_count: int = 0

        super().__init__('arm_debug_node')

        self.joy_sub = self.create_subscription(
            msg_type= Joy,
            topic= self.get_parameter('manipulator_joystick_topic').get_parameter_value().string_value,
            callback= self.record_joy,
            qos_profile= 1
            )
        
        self.joy_sub = self.create_subscription(
            msg_type= ManipulatorControl,
            topic= self.get_parameter('manipulator_joystick_topic').get_parameter_value().string_value,
            callback= self.record_man,
            qos_profile= 1
        )
        
        def save_message_as_csv_line(self, message):
            # Assuming message is an instance of YourMessageType
            shoulder_dps = message.shoulder_dps
            elbow_dps = message.elbow_dps
            linear_rail_dps = message.linear_rail_dps
            wrist_pitch_dps = message.wrist_pitch_dps
            wrist_roll_dps = message.wrist_roll_dps
            restart_motors = message.restart_motors

            # Create a list containing the data fields
            csv_data = [
                shoulder_dps,
                elbow_dps,
                linear_rail_dps,
                wrist_pitch_dps,
                wrist_roll_dps,
                restart_motors
            ]

            # Get current timestamp in the specified format
            timestamp = datetime.now().strftime("%m-%d-%Y %H-%M-%S")

            # Header labels for the columns
            header_labels = [
                "Timestamp",
                "Seq",
                "Stamp_secs",
                "Stamp_nsecs",
                "Frame_ID",
                "Shoulder_dps",
                "Elbow_dps",
                "Linear_rail_dps",
                "Wrist_pitch_dps",
                "Wrist_roll_dps",
                "Restart_motors"
            ]

            # Check if the CSV file exists, if not create it and write the header
            with open('operator_control_data.csv', 'a') as csvfile:
                csv_writer = csv.writer(csvfile)
                if csvfile.tell() == 0:  # If the file is empty, write the header
                    csv_writer.writerow(header_labels)
                csv_data.insert(0, timestamp)  # Insert timestamp at the beginning
                csv_writer.writerow(csv_data)


        def record_joy(self, joy_msg):

            # Extracting data from the Twist message
            linear_x = joy_msg.linear.x
            linear_y = joy_msg.linear.y
            linear_z = joy_msg.linear.z
            angular_x = joy_msg.angular.x
            angular_y = joy_msg.angular.y
            angular_z = joy_msg.angular.z

            # Get current timestamp in the specified format
            timestamp = datetime.now().strftime("%m-%d-%Y %H-%M-%S")

            # Header labels for the columns
            header_labels = [
                "Timestamp",
                "Linear_X",
                "Linear_Y",
                "Linear_Z",
                "Angular_X",
                "Angular_Y",
                "Angular_Z"
            ]

            # Create a list containing the data fields
            csv_data = [
                timestamp,
                linear_x,
                linear_y,
                linear_z,
                angular_x,
                angular_y,
                angular_z
            ]

            # Check if the CSV file exists, if not create it and write the header
            with open('joy_data.csv', 'a') as csvfile:
                csv_writer = csv.writer(csvfile)
                if csvfile.tell() == 0:  # If the file is empty, write the header
                    csv_writer.writerow(header_labels)
                csv_writer.writerow(csv_data)



def main(args=None):
    rclpy.init(args=args)
    manipulator_node = OperatorControlNode()
    rclpy.spin(manipulator_node)
    manipulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
