from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    configuration_basename = 'cartographer.lua'
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    
    cartographer_node = Node(
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', cartographer_config_dir,
                    '-configuration_basename', configuration_basename]
    )

    occupancy_grid_node = Node(
    
        package='cartographer_ros',
        executable='occupancy_grid_node',
        output='screen',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    
    )
    
    rviz_node = Node(
    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('cartographer_slam'), 'config', 'mapper_rviz_config.rviz'])]
    
    )

    return LaunchDescription([
    
            cartographer_node,
            occupancy_grid_node,
            rviz_node
    ])