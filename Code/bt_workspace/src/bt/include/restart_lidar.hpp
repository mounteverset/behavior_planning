#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <boost/algorithm/string.hpp>

#include "behaviortree_cpp_v3/action_node.h"

class RestartLidar : public BT::SyncActionNode
{   
    public:

    RestartLidar(const std::string& name) : BT::SyncActionNode(name, {})
    {
        
    }

    std::string exec(const char* cmd) 
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    BT::NodeStatus tick() override
    {
        std::string error_msg = "error";

        std::string result = exec(cmd);

        boost::to_lower(result);

        if (result.find(error_msg) != std::string::npos)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    const char* cmd = "ros2 launch rplidar_ros rplidar.launch"; 
};