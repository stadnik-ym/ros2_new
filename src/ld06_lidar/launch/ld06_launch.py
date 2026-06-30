#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LD06 lidar'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='230400',
        description='Serial baudrate'
    )

    ld06_node = Node(
        package='ld06_lidar',
        executable='ld06_node',
        name='ld06_node',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')},
            {'baudrate': LaunchConfiguration('baudrate')},
            {'frame_id': 'laser_frame'},
            {'publish_rate_hz': 50.0},
            {'angle_resolution_deg': 1.0},
            {'angle_offset_deg': 0.0},
            {'invert_angle_direction': False},
            {'front_min_deg': -35.0},
            {'front_max_deg': 35.0},
            {'point_max_age_sec': 0.35},
            {'front_min_points': 1},
        ]
    )

    safety_node = Node(
        package='ld06_lidar',
        executable='ld06_safety_node',
        name='lidar_safety',
        output='screen',
        parameters=[
            {'input_cmd_topic': '/cmd_vel_raw'},
            {'output_cmd_topic': '/cmd_vel'},
            {'front_distance_topic': '/lidar/front_distance'},
            {'safety_stop_topic': '/safety/lidar_stop'},
            {'cmd_timeout_sec': 0.5},
            {'lidar_timeout_sec': 0.7},
            {'stop_distance_m': 0.22},
            {'clear_distance_m': 0.30},
            {'invalid_front_timeout_sec': 1.0},
            {'allow_rotation_when_blocked': True},
            {'allow_reverse_when_blocked': True},
            {'max_linear_x': 0.35},
            {'max_angular_z': 1.5},
            {'enable_slowdown': True},
            {'slowdown_distance_m': 0.80},
            {'min_slowdown_factor': 0.25},
        ]
    )

    motor_node = Node(
        package='diff_drive',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        ld06_node,
        safety_node,
        motor_node,
    ])
