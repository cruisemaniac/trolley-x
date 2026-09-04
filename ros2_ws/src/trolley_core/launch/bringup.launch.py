"""Launch the manual drive stack."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Also start the RPLIDAR A1 driver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])),
            launch_arguments={'serial_port': '/dev/lidar'}.items(),
            condition=IfCondition(lidar),
        ),

        Node(package='trolley_core', executable='safety_braking',
             name='safety_braking_node', output='screen',
             parameters=[{'invert_scan': True, 'bearing_offset_deg': 180.0}]),  # lidar mounted flipped
        Node(package='trolley_core', executable='arduino_base',
             name='arduino_base_controller', output='screen'),
        Node(package='trolley_core', executable='cardputer_teleop',
             name='cardputer_teleop', output='screen'),
    ])
