#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace BT
{
    class LidarExecutionCheck : public BT::ConditionNode
    {
        public:
        LidarExecutionCheck(const std::string& name) : BT::ConditionNode(name, {})
        {
            
        }

        BT::NodeStatus tick() override
        {
            std::vector<std::string> node_names = node_pointer->get_node_names();

            bool node_name_found = false;
            std::string lidar_node_name = "rplidar";

            for (int i = 0; i < node_names.size(); i++)
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
    };

    class DecelToMinDrivingSpeed : BT::SyncActionNode
    {
        DecelToMinDrivingSpeed(const std::string& name) : BT::SyncActionNode(name, {})
        {

        }

        BT::NodeStatus tick() override
        {
            
        }
    };

    class CompleteStop : BT::SyncActionNode
    {
        CompleteStop(const std::string& name) : BT::SyncActionNode(name, {})
        {

        }

        BT::NodeStatus tick() override
        {

        }
    };

    class RestartLidar : BT::SyncActionNode
    {
        RestartLidar(const std::string& name) : BT::SyncActionNode(name, {})
        {
            
        }

        BT::NodeStatus tick() override
        {

        }
    };

    class BtHelperNode : rclcpp::Node
    {
        BtHelperNode() : rclcpp::Node("bt_helper_node")
        {

        }
    };

    //extern auto node_ptr = std::make_shared<BtHelperNode>();

    std::shared_ptr<rclcpp::Node> node_pointer = rclcpp::Node::make_shared(bt_helper_node);

    int main(int argc, char* argv[])
    {   
        //node_ptr = std::make_shared<BtHelperNode>();

    };
}
