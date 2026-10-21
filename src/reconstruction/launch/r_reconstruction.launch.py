import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'reconstruction'

    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'r_config.yaml'
    )

    reconstruction_node = Node(
        package=package_name,
        executable='reconstruction_node',
        name='reconstruction_node',
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        reconstruction_node
    ])