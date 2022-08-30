#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_msgs/srv/get_charge.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// #define IDLE_CHARGE_DECREASE_PER_SEC 0.1
// #define DRIVING_CHARGE_DECREASE_PER_MSG 0.02 // 20 msg per second -> 0.4% / sec driving

class RobotBattery : public rclcpp::Node
{
    public:
        RobotBattery(const std::string & node_name);

        ~RobotBattery() = default;

        rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    
    private:


        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

        rclcpp::Service<bt_msgs::srv::GetCharge>::SharedPtr service_;
        void service_callback(
            const bt_msgs::srv::GetCharge_Request::SharedPtr request,
            const bt_msgs::srv::GetCharge_Response::SharedPtr response);

        float charge_;
        // float capacity_;
        float idle_decrease_;
        float drive_decrease_;
             
        bool debug;
};