#include "execution_checker_service.hpp"

ExecutionCheckerService::ExecutionCheckerService (const std::string & node_name) : Node(node_name)
{
    lidar_service_ = this->create_service<std_srvs::srv::SetBool>(
        "lidar_execution_service", 
        std::bind(&ExecutionCheckerService::LidarExecutionCheckServiceCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

    // topics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(); // Transient Local Sub Settings
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ExecutionCheckerService::lidar_callback, this, std::placeholders::_1));
}

ExecutionCheckerService::~ExecutionCheckerService() = default;

void ExecutionCheckerService::LidarExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response)
{   
    time_difference = (this->get_clock()->now().seconds() - last_msg_received_.seconds());

    if(time_difference < MAX_ALLOWED_TIME_DIFFERENCE)
    {
        if(debug_callback)
            RCLCPP_INFO(this->get_logger(), "Lidar active");
        is_lidar_running_ = true;
    }
    else 
    {
        is_lidar_running_ = false;
    }  

    response->message = "";
    response->success = is_lidar_running_;  
}

void ExecutionCheckerService::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{   
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "Received a Scan Message");
        RCLCPP_INFO(this->get_logger(), "last_msg_received at: %f", last_msg_received_.seconds());
        RCLCPP_INFO(this->get_logger(), "msg.header.stamp at: %f", msg->header.stamp.sec);
    }

    last_msg_received_ = this->get_clock()->now();

    // if(debug_callback)
    // {
        // RCLCPP_INFO(this->get_logger(), "Output Port written to: ",  is_lidar_running_);
        // RCLCPP_INFO(this->get_logger(), (is_lidar_running_) ? "true" : "false");
    // }

    // setOutput("is_lidar_running", is_lidar_running_);

    if(debug)
        RCLCPP_INFO(this->get_logger(), "last_msg_time when callback finished: %f", last_msg_received_.seconds());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("ExecutionCheckerService"), "ExecutionCheckerService now active.");

  rclcpp::spin(std::make_shared<ExecutionCheckerService>("execution_checker_service"));

  rclcpp::shutdown();

  return 0;
}
