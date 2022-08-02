#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class CompleteStop : public BT::SyncActionNode
{   
    public:

    CompleteStop(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("complete_stop");
        node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();

        message.linear.x = 0.0;
        message.linear.y = 0.0;
        message.linear.z = 0.0;
        message.angular.z = 0.0;
        message.angular.x = 0.0;
        message.angular.y = 0.0;

        cmd_vel_publisher_->publish(message);

        return BT::NodeStatus::SUCCESS;
    }

    private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};