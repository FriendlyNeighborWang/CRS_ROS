import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('maze_bot_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    # Ensure Gazebo can find the meshes
    install_dir = os.path.join(os.getcwd(), 'install')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        model_path =  install_dir + '/share'

    set_gazebo_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'maze.world')
    xacro_file = os.path.join(pkg_share, 'models', 'maze_bot.xacro')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    # Spawn Robot
    # Spawn at Bottom-Right (approx 1.6, 0.2) to test localization and move to Start (Top-Left)
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'maze_bot', '-topic', 'robot_description', '-x', '1.6', '-y', '0.2', '-z', '0.05', '-Y', '3.14'],
        output='screen'
    )
    
    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'nav2_default_view.rviz')] # Uncomment if you have a config
    )

    return LaunchDescription([
        set_gazebo_model_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_cmd,
        rviz_node
    ])
