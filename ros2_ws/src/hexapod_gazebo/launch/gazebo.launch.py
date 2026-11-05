"""Import the supplied hexapod MJCF conversion; physics starts paused."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_robot(context):
    description_share = get_package_share_directory('hexapod_description')
    use_control = LaunchConfiguration('use_ros2_control').perform(context)
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'hexapod.urdf.xacro'),
        mappings={
            'mesh_prefix': 'file://' + os.path.join(description_share, 'meshes'),
            'use_ros2_control': use_control,
            'controllers_file': LaunchConfiguration('controllers_file').perform(context),
        },
    ).toxml()
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=[
                '-entity', 'hexapod', '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'), '-timeout', '90.0',
            ],
        ),
    ]


def generate_launch_description():
    gazebo_share = get_package_share_directory('hexapod_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='true',
                              description='Pause physics to inspect the uncontrolled model.'),
        DeclareLaunchArgument('use_ros2_control', default_value='false',
                              description='Expose effort interfaces for future policy control.'),
        DeclareLaunchArgument('controllers_file', default_value=os.path.join(
            gazebo_share, 'config', 'controllers.yaml')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.313'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            gazebo_share, 'worlds', 'flat.world')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'pause': LaunchConfiguration('paused'),
                'verbose': 'true',
                'params_file': os.path.join(gazebo_share, 'config', 'gazebo.yaml'),
                'server_required': 'true',
            }.items(),
        ),
        OpaqueFunction(function=launch_robot),
    ])
