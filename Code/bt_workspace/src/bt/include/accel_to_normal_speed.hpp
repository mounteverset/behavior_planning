#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_srvs/srv/set_bool.hpp"

class AccelToNormalSpeed : public BT::SyncActionNode
{   
    public:

    AccelToNormalSpeed(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("accel_to_normal_speed");
        // cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 1);
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "AccelToNormalSpeed Constructor");

        // std::cout << "DecelMinDrivingSpeed Constructor" << std::endl;

        service_client_ = node_->create_client<std_srvs::srv::SetBool>("decel_min_speed_service");
        
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "AccelToNormalSpeed Service Client created.");

        request_ = std::make_shared<std_srvs::srv::SetBool_Request>();

        request_->data = false;

        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Exiting Constructor.");
    }

    virtual ~AccelToNormalSpeed()
    {
        if(debug)
            std::cout << "DecelToMinDrivingSpeed destroyed" << std::endl;
    }

    BT::NodeStatus tick() override
    {   
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "MinDrivingSpeed ticked");

        while (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Decel Speed Service not available, waiting again...");
        }

        auto result = service_client_->async_send_request(request_);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sent Service Request");
        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }

    private:
        bool debug = true;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
        std::shared_ptr<std_srvs::srv::SetBool::Request> request_;
};