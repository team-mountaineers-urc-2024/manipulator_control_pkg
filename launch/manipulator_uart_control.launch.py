from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory


config_dir = os.path.join(get_package_share_directory('manipulator_control_pkg'),('config'))

config = os.path.join(config_dir, 'nate_manipulator_control.yaml')

def generate_launch_description():
    launch_description = LaunchDescription()

    launch_description.add_action(
        LogInfo(msg='[manipulator_uart_control.launch.py]\033[34mLaunching the Joystick Node\033[0m'
        )
    )

    launch_description.add_action(
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
            )
        )

    launch_description.add_action(
        LogInfo(msg='[manipulator_uart_control.launch.py]\033[34mLaunching the Operator Control Node\033[0m'
        )
    )

    launch_description.add_action(
            Node(
                package='manipulator_control_pkg',
                executable='operator_controls',
                name='manipulator_operator_controls_node',
                parameters = [config]

            )
        )

    launch_description.add_action(
        LogInfo(
        msg='[manipulator_uart_control.launch.py]\033[34mLaunching the UART motor Node\033[0m'
        )
    )

    launch_description.add_action(
            Node(
                package='manipulator_control_pkg',
                executable='manipulator_motors_uart_node',
                name='manipulator_motors_node',
                parameters = [config]

            )
        )

    return launch_description
