#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "bt_msgs/srv/pub_cmd_vel.hpp"

class DecelToMinDrivingSpeed : public BT::SyncActionNode
{   
    public:

    DecelToMinDrivingSpeed(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("decel_to_min_driving_speed");
        // cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 1);
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "DecelMinDrivingSpeed Constructor");

        // std::cout << "DecelMinDrivingSpeed Constructor" << std::endl;

        service_client_ = node_->create_client<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service");
        
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "DecelMinDrivingSpeed Service Client created.");

        // request_ = std::make_shared<geometry_msgs::msg::Twist>();
        request_ = std::make_shared<bt_msgs::srv::PubCmdVel_Request>();

        request_->cmd_vel.linear.x = 0.05;
        request_->cmd_vel.linear.y = 0.0;
        request_->cmd_vel.linear.z = 0.0;
        request_->cmd_vel.angular.z = 0.0;
        request_->cmd_vel.angular.x = 0.0;
        request_->cmd_vel.angular.y = 0.0;

        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Twist Msg created.");

        // request_->cmd_vel = *message_;

        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Request Msg created.");
        request_->time_in_seconds = 1.0;

        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Exiting Constructor.");
    }

    virtual ~DecelToMinDrivingSpeed()
    {
        if(debug)
            std::cout << "DecelToMinDrivingSpeed destroyed" << std::endl;
    }

    BT::NodeStatus tick() override
    {   
        // node_= rclcpp::Node::make_shared("decel_to_min_driving_speed");
        // node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);
        // if(debug)
            // std::cout << "DecelMinDrivingSpeed ticked" << std::endl;
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "MinDrivingSpeed ticked");


        // message_ = std::make_shared<geometry_msgs::msg::Twist>();

        // std::cout << message_->linear.x << std::endl;
        
        // message_->linear.x = 0.1;
        // message_->linear.y = 0.0;
        // message_->linear.z = 0.0;
        // message_->angular.z = 0.0;
        // message_->angular.x = 0.0;
        // message_->angular.y = 0.0;

        while (!service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PubCmdVel Service not available, waiting again...");
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

        // cmd_vel_publisher_->publish(*message_);
    }

    private:
        bool debug = true;
        std::shared_ptr<rclcpp::Node> node_;
        // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        // geometry_msgs::msg::Twist::SharedPtr message_;
        rclcpp::Client<bt_msgs::srv::PubCmdVel>::SharedPtr service_client_;
        std::shared_ptr<bt_msgs::srv::PubCmdVel::Request> request_;
};