from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt',
            executable='bt_lidar',
            name='bt_lidar',
            # parameters=[{'use_sim_time': True}]
        )
    ])
