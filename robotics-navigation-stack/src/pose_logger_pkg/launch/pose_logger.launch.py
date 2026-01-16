import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'pose_logger_pkg'

    # Path to the config file
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'pose_logger_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='pose_logger_node',
            name='pose_logger_node',
            output='screen',
            parameters=[config_file]
        )
    ])
