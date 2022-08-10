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

class RestartLidar : public BT::SyncActionNode
{   
    public:

    RestartLidar(const std::string& name) : BT::SyncActionNode(name, {})
    {
        
    }

    // std::string exec(const char* cmd) 
    // {
    //     std::array<char, 128> buffer;
    //     std::string result;
    //     std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    //     if (!pipe) {
    //         throw std::runtime_error("popen() failed!");
    //     }
    //     while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    //         result += buffer.data();
    //     }
    //     return result;
    // }

    BT::NodeStatus tick() override
    {
        // std::string error_msg = "error";

        // std::string result = exec(cmd);

        // boost::to_lower(result);

        // if (result.find(error_msg) != std::string::npos)
        // {
        //     return BT::NodeStatus::SUCCESS;
        // }
        // else 
        // {
        //     return BT::NodeStatus::FAILURE;
        // }
        try
        {
            system("gnome-terminal -e 'sh -c \"ros2 run gazebo_sensor_drivers lidar_driver; exec bash\"'");
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            if(debug)
                RCLCPP_INFO(rclcpp::get_logger("restart_lidar"), "Trying to restart Lidar");
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