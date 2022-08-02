#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"


class LidarExecutionCheck : public BT::ConditionNode
{
    public:

    LidarExecutionCheck(const std::string& name) : BT::ConditionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("lidar_execution_check");
    }

    BT::NodeStatus tick() override
    {
        std::vector<std::string> node_names = node_->get_node_names();

        bool node_name_found = false;
        std::string lidar_node_name = "rplidar";

        for (size_t i = 0; i < node_names.size(); i++)
        {
            lidar_node_name.compare(node_names[i]);
            if(node_name_found)
                break;
        }
            
        if (node_name_found)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::FAILURE;
        }            
    }

    private:
    
    std::shared_ptr<rclcpp::Node> node_;
};