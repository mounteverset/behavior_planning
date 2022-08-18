#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/action_node.h"

class RestartOdom : public BT::SyncActionNode
{   
    public:

    RestartOdom(const std::string& name) : BT::SyncActionNode(name, {})
    {
        
    }

    BT::NodeStatus tick() override
    {
        try
        {
            system("gnome-terminal -e 'sh -c \"ros2 run gazebo_sensor_drivers odom_driver; exec bash\"'");
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            if(debug)
                RCLCPP_INFO(rclcpp::get_logger("restart_Odom"), "Trying to restart Odom");
            return BT::NodeStatus::SUCCESS;
        }
        catch(const std::exception& e)
        {   
            //std::cerr << e.what() << '\n';
            if(debug)
                RCLCPP_ERROR(rclcpp::get_logger("restart_lidar"), e.what());
            return BT::NodeStatus::FAILURE;
        }
    }

    private:
        bool debug = true;
    // const char* cmd = "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"; 
};