import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "visualization"
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "v_dis_bidirectional_config.yaml",
    )

    visualization_node = Node(
        package=package_name,
        executable="visualization_node",
        name="visualization_node",
        parameters=[config_file_path],
        output="screen",
        emulate_tty=True,
    )

    display_node = Node(
        package=package_name,
        executable="interpolation_display_node",
        name="interpolation_display_node",
        parameters=[config_file_path],
        output="screen",
        emulate_tty=True,
    )

    stop_if_visualization_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=visualization_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="visualization_node exited"))
            ],
        )
    )
    stop_if_display_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=display_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="interpolation_display_node exited"))
            ],
        )
    )

    return LaunchDescription([
        stop_if_visualization_exits,
        stop_if_display_exits,
        visualization_node,
        display_node,
    ])
