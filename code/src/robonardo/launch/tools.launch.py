from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robonardo_dir = get_package_share_directory('robonardo')
    turtlebot3_nav2_dir = get_package_share_directory('turtlebot3_navigation2')
    turtlebot3_cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    ros2_aruco_dir = get_package_share_directory('ros2_aruco')

    nav2_params = os.path.join(
        robonardo_dir, 'config', 'nav2-custom-params.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robonardo_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_nav2_dir, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={'params_file': nav2_params}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_cartographer_dir, 'launch', 'cartographer.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros2_aruco_dir, 'launch', 'aruco_recognition.launch.py')
            )
        ),
    ])