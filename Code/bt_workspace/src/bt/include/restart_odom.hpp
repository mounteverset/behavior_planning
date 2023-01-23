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
        if(debug)
                RCLCPP_INFO(rclcpp::get_logger("restart_odom"), "Trying to restart Odometry");
        try
        {
            int i, ret = system("gnome-terminal -e 'sh -c \"bash src/bt/scripts/ssh_microros.sh ; exec bash\"'");

            i = WEXITSTATUS(ret);

            std::this_thread::sleep_for(std::chrono::milliseconds(11000));
            
            if (i == 0)
            {
                return BT::NodeStatus::SUCCESS;
            }
            else 
            {
                return BT::NodeStatus::FAILURE;
            }
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