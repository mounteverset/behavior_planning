# Behavioural Planning

## Installation
Clone this repository

Build the workspace
```
cd ~/behavior-planning/Code/bt_workspace
colcon build --symlink-install 
```

Source each new terminal!


`source install/setup.bash`


or add this to your .bashrc


`source ~/behavior-planning/Code/bt_workspace/install/setup.bash`

For simulation with the Turtlebot clone this repository (either in the same workspace or create a new one):

`git clone --branch galactic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`




## Start-Up
To start the Behavior Planner these ROS2 Nodes need to be launched / available:

For simulation:
```
ros2 run gazebo_sensor_drivers lidar_driver
ros2 run gazebo_sensor_drivers odom_driver
ros2 run gazebo_sensor_drivers imu_driver
```

or alternatively:
`ros2 launch gazebo_sensor_drivers sensor_drivers.launch`

The simulation environment:
`ros2 launch turtlebot3_gazebo turtlebot3_house.launch`

Navigation 2:
`ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time=True map='/home/$USER/map_house.yaml'`

BT Utilities:
```
ros2 run cmd_vel_decision_gate cmd_vel_decision_gate
ros2 run sensor_data_backup sensor_data_backup
ros2 run bt execution_checker_service
ros2 run robot_battery robot_battery
```

or alternatively:
`ros2 launch bt bt_utils.launch.py`

and the Behavior Tree:
`ros2 run bt bt_lidar`

To monitor the live BT state with Groot the ports in the live monitoring are:
```
Server IP: localhost
Publisher Port: 1668
Server Port: 1669
```



## 

