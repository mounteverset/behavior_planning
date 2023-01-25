#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_msgs/srv/get_charge.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

// #define IDLE_CHARGE_DECREASE_PER_SEC 0.1
// #define DRIVING_CHARGE_DECREASE_PER_MSG 0.02 // 20 msg per second -> 0.4% / sec driving

class RobotBattery : public rclcpp::Node
{
    public:
        RobotBattery(const std::string & node_name);

        ~RobotBattery() = default;

        rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
        float read_voltage;
    private:


        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

        void voltage_info(const sensor_msgs::msg::BatteryState::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_voltage_;

        rclcpp::Service<bt_msgs::srv::GetCharge>::SharedPtr service_;
        void service_callback(
            const bt_msgs::srv::GetCharge_Request::SharedPtr request,
            const bt_msgs::srv::GetCharge_Response::SharedPtr response);


        float charge_;

        float total_ToF_wattage_;
        float total_motor_wattage_max_;
        float total_motor_wattage_min_; 
        float total_RBi_wattage_;
        float total_core2_wattage_;
        float safety_factor_;
        float lidar_total_;
        float total_power_;
        float read_voltage_ = 11.1;
        float cmd_vel_x_;
        float battery_SOC;
             
        bool debug;
};