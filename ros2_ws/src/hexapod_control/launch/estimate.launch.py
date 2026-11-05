"""一键启动：暂停生成模型→配置 PD→预热策略→激活并解除暂停。"""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_project_root():
    # 兼容 colcon symlink-install 和普通 install，不依赖运行时 cwd。
    for start in (Path(__file__).resolve(), Path(get_package_share_directory('hexapod_control'))):
        for parent in start.parents:
            if (parent / 'deploy' / 'deploy_real' / 'hexapod_estimate_utils').is_dir():
                return parent
    raise RuntimeError('Cannot locate Hexapod_Sim2Real; run from its ros2_ws installation')


def generate_launch_description():
    root = find_project_root()
    share = Path(get_package_share_directory('hexapod_control'))
    # 本机已存在兼容 ROS Humble 的 Python 3.10 + PyTorch 环境。
    # 用户可传 python_executable:=... 使用项目自己的 venv。
    candidates = [root / '.venv/bin/python', root.parent / 'El4090_sim2real-master/.venv/bin/python']
    python = next((str(path) for path in candidates if path.exists()), sys.executable)
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=['hexapod_pd_controller', '--inactive', '--controller-manager-timeout', '90'])
    controller = Node(
        package='hexapod_control', executable='estimate_controller.py',
        prefix=LaunchConfiguration('python_executable'), output='screen',
        on_exit=Shutdown(reason='Estimate controller exited'),
        parameters=[str(share / 'config' / 'estimate.yaml'), {
            'use_sim_time': True, 'project_root': LaunchConfiguration('project_root'),
            'real_config': LaunchConfiguration('real_config'), 'policy_path': LaunchConfiguration('policy_path'),
            'initial_mode': LaunchConfiguration('initial_mode'),
            'gamepad': ParameterValue(LaunchConfiguration('gamepad'), value_type=bool),
            'log_csv': ParameterValue(LaunchConfiguration('log_csv'), value_type=bool),
        }],
    )

    def after_configure(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='PD controller configuration failed.'), Shutdown(reason='spawner failed')]
        return [controller]

    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=str(root)),
        DeclareLaunchArgument('python_executable', default_value=python),
        DeclareLaunchArgument('real_config', default_value=''),
        DeclareLaunchArgument('policy_path', default_value=''),
        DeclareLaunchArgument('initial_mode', default_value='default'),
        DeclareLaunchArgument('gamepad', default_value='true'),
        DeclareLaunchArgument('log_csv', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_airy', default_value='true'),
        DeclareLaunchArgument('airy_horizontal_samples', default_value='900'),
        DeclareLaunchArgument('airy_vertical_samples', default_value='96'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('hexapod_gazebo')) / 'launch' / 'gazebo.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'), 'paused': 'true', 'use_ros2_control': 'true',
                'controllers_file': str(share / 'config' / 'controllers.yaml'),
                'use_airy': LaunchConfiguration('use_airy'),
                'airy_horizontal_samples': LaunchConfiguration('airy_horizontal_samples'),
                'airy_vertical_samples': LaunchConfiguration('airy_vertical_samples'),
            }.items()),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_configure)),
        spawner,
    ])
