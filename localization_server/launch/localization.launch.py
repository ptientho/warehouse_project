from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename': PathJoinSubstitution([map_file_dir, map_file_f])}
                    ]   
    
    
    )

    nav2_amcl_config_dir = PathJoinSubstitution([get_package_share_directory('localization_server'), 'config'])
    nav2_yaml = PathJoinSubstitution([nav2_amcl_config_dir, 'amcl_config.yaml'])
    amcl_node = Node(
    
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]

    )


    lifecycle_manager_localization = Node(
    
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server','amcl']}]

    )

    rviz2_node = Node(
    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('localization_server'), 'config', 'localization_config.rviz'])]
    
    )

    call_global_localization = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/reinitialize_global_localization ",
                "std_srvs/srv/Empty",
            ]],
            shell=True
            )

    return LaunchDescription([

        map_file_args,
        rviz2_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        call_global_localization
        
    
    
    ])