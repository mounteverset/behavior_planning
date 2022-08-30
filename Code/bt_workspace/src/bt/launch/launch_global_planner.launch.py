import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # rviz_config_dir = os.path.join(get_package_share_directory('nav2_course'), 'config', 'nav2_default_view.rviz')
    # map_file = os.path.join(get_package_share_directory('nav2_course'), 'config', 'turtlebot3_world.yaml')
    params = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'param',
        'waffle.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{params} 
                       ]
            ),
        Node
        (
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=
            [
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['planner_server']}
            ]
        )            
    ])
