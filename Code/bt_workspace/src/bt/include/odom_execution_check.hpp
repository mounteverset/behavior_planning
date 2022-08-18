#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"


class OdomExecutionCheck : public BT::ConditionNode
{
public:

    OdomExecutionCheck(const std::string& name) : BT::ConditionNode(name, {})
    {   
        is_odom_running_ = true;
        debug = true;
        node_= rclcpp::Node::make_shared("odom_execution_check");
        service_client_ = node_->create_client<std_srvs::srv::SetBool>("odom_execution_service");
        request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        request_->data = true;
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Odom Execution Service not available, waiting again...");
        }

        auto result = service_client_->async_send_request(request_);

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            is_odom_running_ = result.get()->success;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }

        // getInput("is_odom_running", is_odom_running_);
        // (is_odom_running_) ? is_odom_running_ = true : is_odom_running_ = false;

        if(debug)
        {
            // RCLCPP_INFO(rclcpp::get_logger("odom_execution_check"), "Input port read as: ");
            RCLCPP_INFO(rclcpp::get_logger("odom_execution_check"), (is_odom_running_) ? "true" : "false");
        }

        if (is_odom_running_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    // static BT::PortsList providedPorts()
    // {
    //     // This action has a single input port called "message"
    //     // Any port must have a name. The type is optional.
    //     return { BT::InputPort<bool>("is_odom_running") };
    // }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
    std::shared_ptr<std_srvs::srv::SetBool::Request> request_;
    bool debug;
    bool is_odom_running_;
};