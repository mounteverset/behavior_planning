#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "rcl_interfaces/srv/set_parameters.hpp"


class EnableCmdVelOverride : public BT::SyncActionNode
{   
    public:

    EnableCmdVelOverride(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("enable_cmd_vel_override");

        service_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>("cmd_vel_decision_gate/set_parameters");

        param.name = "bt_override";
        param.value.type = 1;
        param.value.bool_value = true;      

        

        // param.value.bool_value = true;

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "EnableCmdVelOverride created"); 

    }

    bool set_override_flag_service_call()
    {
        // auto request = service_client_set_override_flag->

        while (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SetParam Service not available, waiting again...");
        }

        
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Waiting for set param service successful");   

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters.insert(request->parameters.begin(), param);
        // request_set_parameter->parameters = parameters;

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Set the parameters request");   

        auto result = service_client_->async_send_request(request);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sent service request");   

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            if (debug)
                RCLCPP_INFO(node_->get_logger(), "SetParam Service success");
            return true;
        }
        else
        {   
            if (debug)
                RCLCPP_INFO(node_->get_logger(), "SetParam Service failure");
            return false;
        }
    }

    BT::NodeStatus tick() override
    {   
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "EnableCmdVelOverride ticked");   

        if(!set_override_flag_service_call())
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    private:
    bool debug = true;
    std::shared_ptr<rclcpp::Node> node_;

    rcl_interfaces::msg::Parameter param = rcl_interfaces::msg::Parameter();
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr service_client_;
    // std::vector<rcl_interfaces::msg::Parameter> parameters;
    
    // std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request_set_parameter = std::shared_ptr<rcl_interfaces::srv::SetParameters::Request>();
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

};