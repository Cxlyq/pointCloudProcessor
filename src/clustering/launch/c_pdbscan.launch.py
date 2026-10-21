#!/home/cx/anaconda3/envs/pointcloud_607_ros2/bin/python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'clustering'

    # 获取 yaml 配置文件的绝对路径
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'c_config.yaml'
    )

    # 定义要启动的节点
    pdbscan_node = Node(
        package=package_name,
        executable='parallel_dbscan_node',  # 必须与 setup.py 注册的名字一致
        name='parallel_dbscan_node',  # 必须与 yaml 中的顶级键名一致
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        pdbscan_node
    ])