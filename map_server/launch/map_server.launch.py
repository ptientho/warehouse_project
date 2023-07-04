from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    map_file_args = DeclareLaunchArgument('map_file', default_value='default_map.yaml', description='map file name in config folder')
    map_file_f = LaunchConfiguration('map_file')

    map_file_dir = PathJoinSubstitution([get_package_share_directory('map_server'), 'config'])

    map_server_node = Node(
    
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': PathJoinSubstitution([map_file_dir, map_file_f])}
                    ]    

    )

    rviz2_node = Node(
    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('map_server'), 'config', 'mapper_rviz_config.rviz'])]
    
    )

    lifecycle_manager_node = Node(
    
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    
    )


    return LaunchDescription([
        map_file_args,
        rviz2_node,
        map_server_node,
        lifecycle_manager_node
    
    ])