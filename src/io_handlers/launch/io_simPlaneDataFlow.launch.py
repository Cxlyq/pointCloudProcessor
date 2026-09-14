import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    """
    使用 OpaqueFunction 在运行时求值启动参数，
    仅当用户显式传入非空值时才覆盖 yaml 中的默认值，
    从而避免空字符串与参数类型（double）的冲突。
    """
    package_name = 'io_handlers'

    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'io_config.yaml'
    )

    # 运行时读取启动参数的实际值
    dataset_dir_val  = LaunchConfiguration('dataset_dir').perform(context)
    publish_rate_val = LaunchConfiguration('publish_rate').perform(context)

    # 从 yaml 开始，仅对用户显式传入的参数进行覆盖
    extra_params: dict = {}
    if dataset_dir_val:
        extra_params['dataset_dir'] = dataset_dir_val
    if publish_rate_val:
        try:
            extra_params['publish_rate'] = float(publish_rate_val)
        except ValueError:
            pass  # 非法值则忽略，沿用 yaml 配置

    # 构建参数列表：yaml 基础配置 + 可选覆盖字典
    parameters = [config_file_path]
    if extra_params:
        parameters.append(extra_params)

    sim_plane_node = Node(
        package=package_name,
        executable='sim_plane_data_flow_node',
        name='sim_plane_data_flow_node',
        parameters=parameters,
        output='screen',
        emulate_tty=True
    )

    return [sim_plane_node]


def generate_launch_description():
    return LaunchDescription([
        # ── 可选的命令行覆盖参数（默认为空，表示不覆盖 yaml）──
        DeclareLaunchArgument(
            'dataset_dir',
            default_value='',
            description='数据集目录路径，需包含 extend_data.json 和 powerLineGPS.json。'
                        '不传则使用 io_config.yaml 中的默认值。'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='',
            description='点云 + 载机状态发布频率 (Hz)。'
                        '不传则使用 io_config.yaml 中的默认值。'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
