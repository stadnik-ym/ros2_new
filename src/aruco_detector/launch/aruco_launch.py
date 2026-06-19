from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        emulate_tty=True,
        respawn=True,
    )

    camera_publisher_node = Node(
        package='aruco_detector',
        executable='camera_publisher_node',
        name='camera_publisher_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
    )

    return LaunchDescription([aruco_detector_node, camera_publisher_node])
