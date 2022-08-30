import os
import launch

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()

    # bt_directory = get_package_share_directory("bt")

    # param_file = os.path.join(bt_directory, "config", "util_params.yaml")

    lidar_cmd = Node(
        package="gazebo_sensor_drivers",
        executable="lidar_driver",
        output="screen",

    )

    imu_cmd = Node(
        package="gazebo_sensor_drivers",
        executable="imu_driver",
        output="screen",

    )

    odom_cmd = Node(
        package="gazebo_sensor_drivers",
        executable="odom_driver",
        output="screen",

    )


    ld.add_action(lidar_cmd)
    ld.add_action(imu_cmd)
    ld.add_action(odom_cmd)

    return ld