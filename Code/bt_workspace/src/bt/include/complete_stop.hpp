#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

class CompleteStop : public BT::SyncActionNode
{   
    public:

    CompleteStop(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("complete_stop");
        //cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);
        // service_client_ = node_->create_client<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service");
        // request_ = std::make_shared<bt_msgs::srv::PubCmdVel_Request>();
        cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 1);

        stop_msg.linear.x = 0.0;
        stop_msg.linear.y = 0.0;
        stop_msg.linear.z = 0.0;
        stop_msg.angular.z = 0.0;
        stop_msg.angular.x = 0.0;
        stop_msg.angular.y = 0.0;
       
    }

    BT::NodeStatus tick() override
    {
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "CompleteStop ticked");
        cmd_vel_publisher_->publish(stop_msg);

        return BT::NodeStatus::SUCCESS;
    }

    private:
    bool debug = false;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    geometry_msgs::msg::Twist stop_msg;
    // rclcpp::Client<bt_msgs::srv::PubCmdVel>::SharedPtr service_client_;
    // std::shared_ptr<bt_msgs::srv::PubCmdVel::Request> request_;
};