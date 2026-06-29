import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'ground_segmentation'

    # 指向 gs_config.yaml
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gs_config.yaml'
    )

    ransac_node = Node(
        package=package_name,
        executable='ransac_node',  # 对应 setup.py 中的注册名称
        name='ransac_node',  # 与 yaml 顶级键名一致
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        ransac_node
    ])