import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    docbot_nav_pkg_dir = get_package_share_directory("docbot_navgation")
    rviz_config_file = os.path.join(docbot_nav_pkg_dir, 'rviz', 'nav2.rviz')


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_filename = LaunchConfiguration('map', default='/home/mr/ros_ws/docbot/src/docbot_navgation/map/save_map.yaml')
    map_subscribe_transient_lcoal = LaunchConfiguration('map_subscribe_transient_local', default='true')


    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(docbot_nav_pkg_dir, "launch", "navigation_launch.py")),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(docbot_nav_pkg_dir, "launch", "localization_launch.py")),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_filename,
            'map_subscribe_transient_lcoal': map_subscribe_transient_lcoal,
            }.items(),
    )



    rviz_node = Node(
        package='rviz2',
        executable ='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )

    twist_stamper_node = Node(
        package='docbot_navgation',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
        navigation_launch,
        localization_launch,
        twist_stamper_node
    ])