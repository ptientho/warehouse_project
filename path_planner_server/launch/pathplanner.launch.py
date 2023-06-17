from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    nav2_config_dir = PathJoinSubstitution([get_package_share_directory('path_planner_server'), 'config'])
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Planner~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    planner_yaml = PathJoinSubstitution([nav2_config_dir, 'planner_server.yaml'])
    planner = Node(
    
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml],
        
        
    )
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Controller~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    controller_yaml = PathJoinSubstitution([nav2_config_dir, 'controller.yaml'])
    controller = Node(
    
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[controller_yaml],
        remappings=[('/cmd_vel','/robot/cmd_vel')]
    
    )
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Bt navigator~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
    bt_navigator_yaml = PathJoinSubstitution([nav2_config_dir, 'bt.yaml'])
    
    behavior_tree_navigator = Node(
    
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml],
        
    )
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Recovery behaviors~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    recovery_yaml = PathJoinSubstitution([nav2_config_dir, 'recovery.yaml'])
    
    manager_recovery_behaviors = Node(
    
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        parameters=[recovery_yaml],
        output='screen',
        remappings=[('/cmd_vel','/robot/cmd_vel')]
        
    )

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Lifecycle manager~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    nav2_lifecycle_manager = Node(
    
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_path_planner',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'recoveries_server',
                                    'bt_navigator']}
                    ],
    )
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Rviz 2~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    rviz2_node = Node(
    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('path_planner_server'), 'config', 'pathplanning.rviz'])]
    
    )
    
    return LaunchDescription([
        
        rviz2_node,
        controller,
        planner,
        manager_recovery_behaviors,
        behavior_tree_navigator,
        nav2_lifecycle_manager,
        
    ])