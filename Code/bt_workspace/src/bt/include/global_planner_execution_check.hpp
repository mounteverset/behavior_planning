#include "behaviortree_cpp_v3/condition_node.h"

#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"


class GlobalPlannerExecutionCheck : public BT::ConditionNode
{
public:

    GlobalPlannerExecutionCheck(const std::string& name) : BT::ConditionNode(name, {})
    {   
        is_planner_running_ = true;
        debug = true;
        node_= rclcpp::Node::make_shared("global_planner_execution_check");
        service_client_ = node_->create_client<lifecycle_msgs::srv::GetState>("planner_server/get_state");
        // request_ = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Global Planner Execution Service not available, waiting again...");
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        auto result = service_client_->async_send_request(request);

        lifecycle_msgs::msg::State state;

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            state = result.get()->current_state;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }

        if (state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            is_planner_running_ = true;
        }
        else
        {
            is_planner_running_ = false;
        }

        if(debug)
        {
            RCLCPP_INFO(node_->get_logger(), "Is Global Planner running: ");
            RCLCPP_INFO(node_->get_logger(), (is_planner_running_) ? "true" : "false");
        }

        if (is_planner_running_)
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
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr service_client_;
    bool debug;
    bool is_planner_running_;
};