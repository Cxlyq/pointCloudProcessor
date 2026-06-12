import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 io_handlers 包的共享目录
    package_name = 'io_handlers'

    # 拼接配置文件的绝对路径
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'io_config.yaml'
    )

    # 定义节点
    io_handler_node = Node(
        package=package_name,
        executable='sim_lidar_data_flow_node',
        name='sim_lidar_data_flow_node',
        parameters=[config_file_path],  # 注入 YAML 参数
        output='screen',  # 将日志输出到终端
        emulate_tty=True  # 保证彩色日志输出
    )

    return LaunchDescription([
        io_handler_node
    ])