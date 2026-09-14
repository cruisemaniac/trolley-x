"""Autonomous follow stack: lidar + safety + base + UWB + EKF + follow + logger.

follow_controller -> /motion_request -> safety_braking -> /cmd_vel -> arduino_base.
Do not run teleop at the same time (both publish /motion_request).

GEOMETRY - the three values below are ESTIMATES (never measured; wiring.md marks
them "TODO"). The cart will drive and follow with these, but replace them with a
tape measurement when you can:
  wheel_track_m   : distance between the LEFT and RIGHT wheel contact points (m).
                    Low sensitivity - skid-steer slips, so this is a turn-rate
                    scale you tune anyway via k_bearing / min_turn_pwm. Estimate ok.
  anchor_x        : forward distance from base_link origin to the front pillars
                    where the anchors sit (m). Low sensitivity at follow ranges.
  anchor_baseline : LEFT-to-RIGHT separation of the two front-pillar anchors (m).
                    THIS ONE MATTERS - it sets the EKF bearing scale. Measure it.
                    Wider = sharper bearing (0.30 m at ~2 m is weak).
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
    log = LaunchConfiguration('log')

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Also start the RPLIDAR A1 driver.'),
        DeclareLaunchArgument('log', default_value='true',
                              description='Also start the CSV data logger.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py'])),
            launch_arguments={'serial_port': '/dev/lidar'}.items(),
            condition=IfCondition(lidar),
        ),

        Node(package='trolley_core', executable='safety_braking',
             name='safety_braking_node', output='screen',
             parameters=[{'invert_scan': True, 'bearing_offset_deg': 180.0}]),
        Node(package='trolley_core', executable='arduino_base',
             name='arduino_base_controller', output='screen',
             parameters=[{'wheel_track_m': 0.35}]),        # ESTIMATE - measure and replace
        Node(package='trolley_core', executable='uwb_ranging',
             name='uwb_ranging', output='screen',
             parameters=[{'right_offset_m': -0.23,   # measured: right read +0.23 m long
                          'left_offset_m': 0.0}]),
        Node(package='trolley_core', executable='uwb_localizer',
             name='uwb_localizer', output='screen',
             parameters=[{'anchor_x': 0.22,                # MEASURED: anchors 22 cm forward of cart centre
                          'anchor_baseline': 0.24,         # MEASURED tip-to-tip (2026-09-14)
                          'process_accel_std': 0.6}]),
        Node(package='trolley_core', executable='follow_controller',
             name='follow_controller', output='screen',
             parameters=[{'use_target': True,
                          'standoff_m': 0.45,
                          'range_offset_m': 0.22}]),   # = anchor_x; gap = EKF range - this
        Node(package='trolley_core', executable='follow_logger',
             name='follow_logger', output='screen',
             parameters=[{'standoff_m': 0.45}],
             condition=IfCondition(log)),
    ])
