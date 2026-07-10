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

    # 启动新的混合重建节点
    reconstruction_poisson_node = Node(
        package=package_name,
        executable='reconstruction_poisson_node', # 需在 CMakeLists 中配置同名执行文件
        name='reconstruction_poisson_node',       # 对应 yaml 中的新建命名空间
        parameters=[config_file_path],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        reconstruction_poisson_node
    ])