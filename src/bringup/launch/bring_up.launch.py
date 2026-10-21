import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_name = 'bringup'
    pkg_share = get_package_share_directory(package_name)

    config_file = LaunchConfiguration('config_file').perform(context).strip()
    board = LaunchConfiguration('board').perform(context).strip()
    log_level = LaunchConfiguration('log_level').perform(context).strip()
    show_output = LaunchConfiguration('show_output').perform(context).strip()

    # 根据参数选择配置文件优先级：config_file > board 预设 > 默认单机 full 配置
    if config_file:
        selected_config = config_file
    elif board in ('board1', '1'):
        selected_config = os.path.join(pkg_share, 'config', 'bring_up_board1.yaml')
    elif board in ('board2', '2'):
        selected_config = os.path.join(pkg_share, 'config', 'bring_up_board2.yaml')
    else:
        selected_config = os.path.join(pkg_share, 'config', 'bring_up.yaml')

    params = [selected_config]
    overrides = {}
    if log_level:
        overrides['global_log_level'] = log_level.upper()
    if show_output:
        overrides['global_show_output'] = (show_output.lower() in ('true', '1', 'yes'))
    if overrides:
        params.append(overrides)

    bringup_node = Node(
        package=package_name,
        executable='bringup_node',
        name='bringup_node',
        parameters=params,
        output='screen',
        emulate_tty=True
    )

    return [bringup_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to custom config file (YAML). If specified, takes precedence over board preset.'
        ),
        DeclareLaunchArgument(
            'board',
            default_value='',
            description='Board preset: "board1" (or "1") for Board 1, "board2" (or "2") for Board 2, or empty for all nodes.'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='',
            description='Global log filter level override: "INFO", "WARN", "ERROR", "NONE"'
        ),
        DeclareLaunchArgument(
            'show_output',
            default_value='',
            description='Global terminal output toggle override: "true" or "false"'
        ),
        OpaqueFunction(function=launch_setup)
    ])
