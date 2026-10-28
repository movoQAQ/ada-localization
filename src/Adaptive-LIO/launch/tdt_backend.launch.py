"""Adaptive-LIO plus the independent T-DT legacy backend (no ZLIO frontend)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    adaptive = get_package_share_directory('adaptive_lio')
    backend = get_package_share_directory('backend')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('frontend_config', default_value=os.path.join(adaptive, 'config', 'mapping_m.yaml')),
        DeclareLaunchArgument('backend_config', default_value=os.path.join(adaptive, 'config', 'tdt_backend.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(adaptive, 'launch', 'run.launch.py')),
                                 launch_arguments={'publish_backend_frames': 'true', 'rviz': 'false',
                                                   'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                   'adaptive_lio_cfg_dir': LaunchConfiguration('frontend_config')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(backend, 'launch', 'backend.launch.py')),
                                 launch_arguments={'config_file': LaunchConfiguration('backend_config'), 'mode': LaunchConfiguration('mode'), 'rviz': 'false',
                                                   'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
    ])
