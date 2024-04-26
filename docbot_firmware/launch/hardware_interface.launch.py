import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    docbot_description_dir = get_package_share_directory("docbot_description")
    robot_description = ParameterValue(Command
        ([
            "xacro ", 
            os.path.join(docbot_description_dir, "urdf", "docbot.urdf.xacro")
        ]),
        value_type =str)
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},

             os.path.join(
                 get_package_share_directory("docbot_controller"),
                 "config",
                 "docbot_controllers.yaml"
             )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
    ])