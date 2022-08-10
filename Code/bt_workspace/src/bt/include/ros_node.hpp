#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ROSHelperNode : public rclcpp::Node
{
    public: 
    ROSHelperNode() : Node("lidar_driver")
    {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 1);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};