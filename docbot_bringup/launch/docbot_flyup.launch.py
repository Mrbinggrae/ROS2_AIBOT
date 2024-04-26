import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("docbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("docbot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("docbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
    )

    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
    ])