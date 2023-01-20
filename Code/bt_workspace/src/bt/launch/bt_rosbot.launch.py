from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt',
            executable='bt_rosbot',
            name='bt_rosbot',
            # parameters=[{'use_sim_time': True}]
        )
    ])
