import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    pkg_share_dir = get_package_share_directory('docbot_navgation')
    rviz_config_file = os.path.join(pkg_share_dir, 'rviz', 'mapping.rviz')

    #tf
    tf2_node = Node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_pub',
                arguments=['0', '0', '0', '0', '0', '0', '1','map','base_footprint'],
                # x, y, z, roll, pitch, yaw  
                )

    #rviz
    rviz2_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )
    
    # toolbox
    toolbox_param_file = LaunchConfiguration('toolbox_params')
    toolbox_params_declare = DeclareLaunchArgument('toolbox_params',
                                                   default_value=os.path.join(pkg_share_dir, 'config', 'mapper_params_online_async.yaml'),
                                                   description='toolbox')
    slam_toolbox_node = Node(
        package='slam_toolbox', 
        executable='async_slam_toolbox_node',
        parameters=[
            toolbox_param_file
        ]
    )


    return LaunchDescription([
        tf2_node,
        rviz2_node,
        toolbox_params_declare,
        slam_toolbox_node,
    ])

