from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    nav2_bringup_pkg_dir = FindPackageShare('nav2_bringup')
    jackal_nav_pkg_dir = FindPackageShare('jackal_nav')
    
    # Map parameters
    map_yaml_file = PathJoinSubstitution([jackal_nav_pkg_dir, 'maps', 'map_1744446424.yaml'])
    
    # Launch configurations
    params_file = PathJoinSubstitution([jackal_nav_pkg_dir, 'config', 'nav2_params.yaml'])
    
    # Include Nav2 bringup
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_pkg_dir, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'true'
        }.items()
    )
    
    # Add RVIZ with navigation config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen')
    
    map_to_odom_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    return LaunchDescription([
        nav2_bringup_cmd,
        rviz_cmd,
        map_to_odom_transform
    ])