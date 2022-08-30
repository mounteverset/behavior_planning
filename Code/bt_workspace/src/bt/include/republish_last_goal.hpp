#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"

/**
 * @brief Republish the last known goal
 * 
 */
class RepublishLastGoal : public BT::SyncActionNode
{
public:

    RepublishLastGoal(const std::string& name) : BT::SyncActionNode(name, {})
    {   
        node_= rclcpp::Node::make_shared("republish_last_goal");
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor RepublishLastGoal");

        service_client_ = node_->create_client<std_srvs::srv::Empty>("pub_last_goal_service");
        request_ = std::make_shared<std_srvs::srv::Empty::Request>();

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor RepublishLastGoal finished.");
        // debug = true;
        debug = false;
    }

    BT::NodeStatus tick() override
    {

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "RepublishLastGoal ticked.");

        while (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RepublishLastGoal Service not available, waiting again...");
        }
        
        auto result = service_client_->async_send_request(request_);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sending Service Request.");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }


        return BT::NodeStatus::FAILURE;
    }


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr service_client_;
    std::shared_ptr<std_srvs::srv::Empty::Request> request_;
    bool debug;

};