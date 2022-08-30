#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"

/**
 * @brief Returns Success when all 4 wheels are still on the ground 
 * Determined by the robots imu orientation,
 * if the terrain contains slopes this method does not work 
 * 
 */
class PathPossibleCheck : public BT::ConditionNode
{
public:

    PathPossibleCheck(const std::string& name) : BT::ConditionNode(name, {})
    {   
        node_= rclcpp::Node::make_shared("path_possible_check");
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor PathPossibleCheck");
        is_path_possible = true;
        service_client_ = node_->create_client<std_srvs::srv::SetBool>("path_possible_check_service");
        request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        request_->data = true;
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor PathPossibleCheck finished.");
        // debug = true;
        debug = true;
    }

    BT::NodeStatus tick() override
    {
        while (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PathPossibleCheck Service not available, waiting again...");
        }
        
        auto result = service_client_->async_send_request(request_);

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            is_path_possible = result.get()->success;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }

        // getInput("is_lidar_running", is_lidar_running_);
        // (is_lidar_running_) ? is_lidar_running_ = true : is_lidar_running_ = false;

        if(debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Calculated Path can be followed: ");
            RCLCPP_INFO(node_->get_logger(), (is_path_possible) ? "true" : "false");
        }

        if (is_path_possible)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
    std::shared_ptr<std_srvs::srv::SetBool::Request> request_;
    bool debug;
    bool is_path_possible;
};