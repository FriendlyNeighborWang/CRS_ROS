import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get directories
    maze_bot_sim_share = get_package_share_directory('maze_bot_sim')
    yahboomcar_nav_share = get_package_share_directory('yahboomcar_nav')
    
    # Define arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Note: yahboomcar_nav uses 'maps' argument, not 'map'
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(maze_bot_sim_share, 'maps', 'maze_stage2.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(maze_bot_sim_share, 'params', 'nav2_params.yaml'))
    
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
            
        # Include the yahboomcar_nav launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yahboomcar_nav_share, 'launch', 'navigation_dwb_launch.py')
            ),
            launch_arguments={
                'maps': map_yaml_path,      # Pass our map to their 'maps' argument
                'use_sim_time': use_sim_time,
                'params_file': params_file, # Pass our params to their 'params_file' argument
                'namespece': ''             # Default empty namespace
            }.items()
        )
    ])
