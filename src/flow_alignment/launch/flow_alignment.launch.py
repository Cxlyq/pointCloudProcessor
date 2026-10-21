# flow_alignment 独立启动文件
# 用法：ros2 launch flow_alignment flow_alignment.launch.py
# 不修改任何已有 launch / 节点；此 launch 独立启动配准节点，
# 订阅 /reconstruction/white_mesh，发布 /flow/aligned_mesh 等。
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('flow_alignment')
    params_file = os.path.join(pkg_share, 'config', 'flow_alignment_params.yaml')

    return LaunchDescription([
        Node(
            package='flow_alignment',
            executable='flow_alignment_node',
            name='flow_alignment_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
