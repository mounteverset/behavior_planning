#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class DecelToMinDrivingSpeed : public BT::SyncActionNode
{   
    public:

    DecelToMinDrivingSpeed(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("decel_to_min_driving_speed");
        node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);
    }

    BT::NodeStatus tick() override
    {   
        node_= rclcpp::Node::make_shared("decel_to_min_driving_speed");
        node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);
        message_ = std::make_shared<geometry_msgs::msg::Twist>();

        // message_->linear.x = 0.1;
        // message_->linear.y = 0.0;
        // message_->linear.z = 0.0;
        // message_->angular.z = 0.0;
        // message_->angular.x = 0.0;
        // message_->angular.y = 0.0;

        cmd_vel_publisher_->publish(*message_);

        return BT::NodeStatus::SUCCESS;
    }

    private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    geometry_msgs::msg::Twist::SharedPtr message_;
};