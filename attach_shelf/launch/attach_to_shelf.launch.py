from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python import get_package_share_directory

def generate_launch_description():

    obstacle_arg = DeclareLaunchArgument("obstacles", default_value='0.0')
    degree_arg =  DeclareLaunchArgument("degrees", default_value='0')
    final_approach_arg = DeclareLaunchArgument("final_approach", default_value='False')

    obstacle_f = LaunchConfiguration('obstacles')
    degree_f = LaunchConfiguration('degrees')
    final_f = LaunchConfiguration('final_approach')

    service_server_node = Node(
        package='attach_shelf',
        executable='attach_shelf_node',
        output='screen',
        name='attach_shelf_server',
        parameters=[{'use_sim_time': True}]
    
    )

    client_node =  Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2_node',
        #arguments=["-obstacles", obstacle_f, "-degrees", degree_f, "-final_approach", final_f]
        parameters=[{
            'obstacles':obstacle_f,
            'degrees':degree_f,
            'final_approach':final_f
        }]

    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('attach_shelf'), 'rviz', 'attach_shelf_vis.rviz'])]
    )

    return LaunchDescription([
        
        obstacle_arg,
        degree_arg,
        final_approach_arg,
        service_server_node,
        client_node
   
    ])