"""Bring up all sensors, no motion source.

Starts the LiDAR, the Arduino base (for IMU + odometry telemetry) and the UWB
ranging + EKF localizer. No follow controller and no teleop, so nothing commands
the wheels. Use it with scripts/test_sensors.py to health-check every sensor.
"""

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

        Node(package='trolley_core', executable='arduino_base',
             name='arduino_base_controller', output='screen'),
        Node(package='trolley_core', executable='uwb_ranging',
             name='uwb_ranging', output='screen',
             parameters=[{'right_offset_m': -0.20,   # calibrated 2026-09-16 static sweep; raw right reads ~0.20 m long
                          'left_offset_m': -0.04}]),   # calibrated 2026-09-16 static sweep; left reads +0.04 m long
        Node(package='trolley_core', executable='uwb_localizer',
             name='uwb_localizer', output='screen',
             parameters=[{'anchor_x': 0.22, 'anchor_baseline': 0.24, 'max_valid_m': 3.5}]),
    ])
