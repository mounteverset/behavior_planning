import os
import launch

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()

    bt_directory = get_package_share_directory("bt")

    param_file = os.path.join(bt_directory, "config", "util_params.yaml")

    decision_gate_cmd = Node(
        package="cmd_vel_decision_gate",
        executable="cmd_vel_decision_gate",
        output="screen",
        parameters=[param_file]
    )

    # robot_battery_cmd = Node(
    #     package="robot_battery",
    #     executable="robot_battery",
    #     output="screen",
    #     parameters=[{"charge": 100.0},
	# 	            {"idle_decrease": 0.01},
	# 	            {"drive_decrease": 0.02}]
    # )

    robot_battery_cmd = Node(
        package="robot_battery",
        executable="robot_battery",
        output="screen",
        parameters=[param_file]
    )

    execution_checker_cmd = Node(
        package="bt",
        executable="execution_checker_service",
        output="screen",
        parameters=[param_file]
    )

    sensor_data_cmd = Node(
        package="sensor_data_backup",
        executable="sensor_data_backup",
        output="screen",
        parameters=[param_file]
    )

    imu_collision_cmd = Node(
        package="imu_collision",
        executable="imu_collision",
        output="screen",
        parameters=[param_file]
    )

    ld.add_action(imu_collision_cmd)
    ld.add_action(decision_gate_cmd)
    ld.add_action(robot_battery_cmd)
    ld.add_action(execution_checker_cmd)
    ld.add_action(sensor_data_cmd)

    return ld