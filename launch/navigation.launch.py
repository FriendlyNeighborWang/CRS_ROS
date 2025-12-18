import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'maze_stage1.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'params', 'nav2_params.yaml'))
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to param file to load'),
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        )
    ])
