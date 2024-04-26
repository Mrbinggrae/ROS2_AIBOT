import os
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # 1. robot_state_publisher의 파라미터 값을 설정하기 위한 명령어를 만든다.
    
    ## 1-1 urdf 파일의 절대 경로
    pkg_dir = get_package_share_directory("docbot_description")
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=
            os.path.join(pkg_dir, "urdf", "docbot.urdf.xacro"),
        description="docbot.urdf.xacro의 절대 경로를 받아온다."
    )

    ## 1-2 터미널 명령어 만들기
    ### $( xacro /home/mr/mr_home/docbot_ws/src/docbot_description/urdf/docbot.urdf.xacro)
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)


    # 2. 노드 만들기
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("docbot_description"), "rviz", "display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui, 
        rviz_node
    ])


    
