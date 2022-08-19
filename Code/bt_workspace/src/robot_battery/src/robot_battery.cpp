#include "robot_battery.hpp"

RobotBattery::RobotBattery(const std::string & node_name) : Node(node_name)
{
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        10, 
        std::bind(&RobotBattery::cmd_vel_callback, 
        this, 
        std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&RobotBattery::timer_callback, 
        this));

    service_ = this->create_service<bt_msgs::srv::GetCharge>(
        "get_charge_service", 
        std::bind(&RobotBattery::service_callback, 
        this, std::placeholders::_1, 
        std::placeholders::_2));
    
    this->declare_parameter("capacity", 100.0);
    this->declare_parameter("charge", 100.0);
    this->declare_parameter("idle_decrease", 0.01);
    this->declare_parameter("drive_decrease", 0.02);

    this->get_parameter("capacity", capacity_);
    this->get_parameter("charge", charge_);
    this->get_parameter("idle_decrease", idle_decrease_);
    this->get_parameter("drive_decrease", drive_decrease_);

    debug = false;
}

void RobotBattery::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(msg->linear.x != 0)
    {
        charge_ -= drive_decrease_;
    }
}

void RobotBattery::timer_callback()
{
    charge_ -=idle_decrease_;

}

void RobotBattery::service_callback(
    const bt_msgs::srv::GetCharge_Request::SharedPtr request,
    const bt_msgs::srv::GetCharge_Response::SharedPtr response)
{
    response->charge = charge_;
    response->idle_decrease_per_sec = idle_decrease_;
    response->drive_decrease_per_sec = drive_decrease_ * 20; // getting 20 cmd_vel msg / sec
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("RobotBattery"), "RobotBattery now active.");

  rclcpp::spin(std::make_shared<RobotBattery>("robot_battery"));

  rclcpp::shutdown();

  return 0;
}
