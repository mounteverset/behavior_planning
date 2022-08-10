#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"


class LidarExecutionCheck : public BT::ConditionNode
{
public:

    LidarExecutionCheck(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
    {   
        is_lidar_running_ = true;
        debug = true;
        node_= rclcpp::Node::make_shared("execution_checker");
        service_client_ = node_->create_client<std_srvs::srv::SetBool>("lidar_execution_service");
        request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        request_->data = true;
    }

    BT::NodeStatus tick() override
    {
        auto result = service_client_->async_send_request(request_);

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            is_lidar_running_ = result.get()->success;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }

        // getInput("is_lidar_running", is_lidar_running_);
        // (is_lidar_running_) ? is_lidar_running_ = true : is_lidar_running_ = false;

        if(debug)
        {
            RCLCPP_INFO(rclcpp::get_logger("lidar_execution_check"), "Input port read as: ");
            RCLCPP_INFO(rclcpp::get_logger("lidar_execution_check"), (is_lidar_running_) ? "true" : "false");
        }

        if (is_lidar_running_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { BT::InputPort<bool>("is_lidar_running") };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
    std::shared_ptr<std_srvs::srv::SetBool::Request> request_;
    bool debug;
    bool is_lidar_running_;
};