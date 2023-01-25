#include "robot_battery.hpp"

RobotBattery::RobotBattery(const std::string & node_name) : Node(node_name)
{
    debug = false;

    subscription_voltage_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery", 10, std::bind(&RobotBattery::voltage_info, this, std::placeholders::_1));

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

    total_ToF_wattage_ = 4*20/1000; 
    total_motor_wattage_max_ = 4*6*1.5;
    total_motor_wattage_min_ = 4*6*0.25;
    total_RBi_wattage_ = 4;
    total_core2_wattage_ = 3;
    safety_factor_ = 1.2;
    lidar_total_ = 1.4;
    battery_SOC = 100;

    // this->declare_parameter("capacity", 100.0);
    this->declare_parameter("charge", 100.0);

    // this->get_parameter("capacity", capacity_);
    this->get_parameter("charge", charge_);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&RobotBattery::parameters_callback, this, std::placeholders::_1));

}

rcl_interfaces::msg::SetParametersResult RobotBattery::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    if(debug)
        RCLCPP_INFO(this->get_logger(), "Parameter Callback called.");

    for (auto &param : parameters)
    {

        RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());

        if(param.get_name() == "charge")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                charge_ = param.as_double();

                if(debug)
                    RCLCPP_INFO(this->get_logger(), "Changed charge.");
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    return result;
}

void RobotBattery::voltage_info(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    this->read_voltage_ = msg->voltage;
    //RCLCPP_INFO(this->get_logger(), "Voltage: '%f'", read_voltage);
}

void RobotBattery::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
   
    this->cmd_vel_x_ = abs(msg->linear.x);
    
}

void RobotBattery::timer_callback()
{
    this->total_power_ = (total_ToF_wattage_ + lidar_total_ + (((total_motor_wattage_max_ + total_motor_wattage_min_)/2) * cmd_vel_x_) + total_RBi_wattage_ + total_core2_wattage_) * safety_factor_;
    if(cmd_vel_x_ == 0){
        this->total_power_ = (total_ToF_wattage_ + lidar_total_ + total_RBi_wattage_/2 + total_core2_wattage_/2) * safety_factor_;
    }
        
    battery_SOC= battery_SOC + (-(this->total_power_/this->read_voltage_)/3.5) * (1/3600.0)*100;
    charge_ = battery_SOC;
    //RCLCPP_INFO(this->get_logger(), "battery percentage discharging: '%f', '%f'", charge_, total_power_);

}

void RobotBattery::service_callback(
    const bt_msgs::srv::GetCharge_Request::SharedPtr request,
    const bt_msgs::srv::GetCharge_Response::SharedPtr response)
{
    response->charge = charge_;
    //response->idle_decrease_per_sec = idle_decrease_;
    //response->drive_decrease_per_sec = drive_decrease_ * 20; // getting 20 cmd_vel msg / sec
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("RobotBattery"), "RobotBattery now active.");

  rclcpp::spin(std::make_shared<RobotBattery>("robot_battery"));

  rclcpp::shutdown();

  return 0;
}
