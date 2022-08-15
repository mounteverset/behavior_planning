#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "bt_msgs/srv/pub_cmd_vel.hpp"

class CompleteStop : public BT::SyncActionNode
{   
    public:

    CompleteStop(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("complete_stop");
        //cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);
        service_client_ = node_->create_client<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service");
        request_ = std::make_shared<bt_msgs::srv::PubCmdVel_Request>();

        request_->cmd_vel.linear.x = 0.00;
        request_->cmd_vel.linear.y = 0.0;
        request_->cmd_vel.linear.z = 0.0;
        request_->cmd_vel.angular.z = 0.0;
        request_->cmd_vel.angular.x = 0.0;
        request_->cmd_vel.angular.y = 0.0;
        request_->time_in_seconds = 0.01;
    }

    BT::NodeStatus tick() override
    {
        // geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();

        // message.linear.x = 0.0;
        // message.linear.y = 0.0;
        // message.linear.z = 0.0;
        // message.angular.z = 0.0;
        // message.angular.x = 0.0;
        // message.angular.y = 0.0;

        // cmd_vel_publisher_->publish(message);
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "CompleteStop ticked");

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
    bool debug = false;
    std::shared_ptr<rclcpp::Node> node_;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Client<bt_msgs::srv::PubCmdVel>::SharedPtr service_client_;
    std::shared_ptr<bt_msgs::srv::PubCmdVel::Request> request_;
};