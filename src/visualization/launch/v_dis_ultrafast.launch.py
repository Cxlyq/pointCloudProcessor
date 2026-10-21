import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "visualization"
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "v_dis_ultrafast_config.yaml",
    )

    visualization_node = Node(
        package=package_name,
        executable="visualization_node",
        name="visualization_node",
        parameters=[config_file_path],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([visualization_node])
