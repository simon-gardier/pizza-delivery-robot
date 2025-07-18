from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robonardo',
            executable='aruco',
            name='aruco_node'
        ),
        Node(
            package='robonardo',
            executable='delivery',
            name='delivery_node'
        ),
        Node(
            package='robonardo',
            executable='explorer',
            name='explorer_node'
        ),
    ])