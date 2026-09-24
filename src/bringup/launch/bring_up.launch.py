import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'bringup'

    # 获取 config/bring_up.yaml 配置文件的绝对路径
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'bring_up.yaml'
    )

    # 定义要启动的总起节点
    bringup_node = Node(
        package=package_name,
        executable='bringup_node',
        name='bringup_node',
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        bringup_node
    ])
