#include "execution_checker_service.hpp"

ExecutionCheckerService::ExecutionCheckerService (const std::string & node_name) : Node(node_name)
{
    lidar_service_ = this->create_service<std_srvs::srv::SetBool>(
        "lidar_execution_service", 
        std::bind(&ExecutionCheckerService::LidarExecutionCheckServiceCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

    imu_service_ = this->create_service<std_srvs::srv::SetBool>(
        "imu_execution_service", 
        std::bind(&ExecutionCheckerService::ImuExecutionCheckServiceCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

    odom_service_ = this->create_service<std_srvs::srv::SetBool>(
        "odom_execution_service", 
        std::bind(&ExecutionCheckerService::OdomExecutionCheckServiceCallback, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

    collision_service_ = this->create_service<std_srvs::srv::SetBool>(
        "collision_check_service",
        std::bind(&ExecutionCheckerService::collision_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    
    orientation_service_ = this->create_service<std_srvs::srv::SetBool>(
        "orientation_check_service",
        std::bind(&ExecutionCheckerService::orientation_checker_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    // topics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(); // Transient Local Sub Settings
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ExecutionCheckerService::lidar_callback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&ExecutionCheckerService::imu_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ExecutionCheckerService::odom_callback, this, std::placeholders::_1));
    sub_collision_ = this->create_subscription<gazebo_msgs::msg::ContactsState>("bumper_states", 10, std::bind(&ExecutionCheckerService::collision_callback, this, std::placeholders::_1));

    debug = false;
    debug_orientation = true;
}

ExecutionCheckerService::~ExecutionCheckerService() = default;

// Service and Topic Callbacks

//Lidar

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


// IMU 
void ExecutionCheckerService::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    last_msg_received_imu_ = this->get_clock()->now();
    
    // Orientation Check
    tf2::Quaternion quat(msg->orientation.x,
                        msg->orientation.y,
                        msg->orientation.z,
                        msg->orientation.w);
    
    tf2::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll, pitch, yaw);

    // if(debug_orientation)
    // {
    //     RCLCPP_INFO(this->get_logger(), "IMU Roll: %f", roll);
    //     RCLCPP_INFO(this->get_logger(), "IMU Pitch: %f", pitch);
    // }

    // if(debug_orientation)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Orientation Checker says all 4 wheels on the ground: ");
    //     RCLCPP_INFO(this->get_logger(), all_wheels_on_the_ground ? "true" : "false");
        
    // }

    if(roll > -0.1 && roll < 0.1 && pitch > -0.1 && pitch < 0.1) //Robot is standing parallel to the ground within some margin (~6°)
    {
        all_wheels_on_the_ground = true;
    }
    else
    {
        all_wheels_on_the_ground = false;
    }

}

void ExecutionCheckerService::ImuExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response)
{
    time_difference_imu = (this->get_clock()->now().seconds() - last_msg_received_imu_.seconds());

    if(time_difference_imu < MAX_ALLOWED_TIME_DIFFERENCE)
    {
        if(debug_callback)
            RCLCPP_INFO(this->get_logger(), "IMU active");
        is_imu_running_ = true;
    }
    else 
    {
        is_imu_running_ = false;
    }  

    response->message = "";
    response->success = is_imu_running_;  
}


// Odom
void ExecutionCheckerService::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_msg_received_odom_ = this->get_clock()->now();
}

void ExecutionCheckerService::OdomExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response)
{
    time_difference_odom = (this->get_clock()->now().seconds() - last_msg_received_odom_.seconds());

    if(time_difference_odom < MAX_ALLOWED_TIME_DIFFERENCE)
    {
        if(debug_callback)
            RCLCPP_INFO(this->get_logger(), "Odom active");
        is_odom_running_ = true;
    }
    else 
    {
        is_odom_running_ = false;
    }  

    response->message = "";
    response->success = is_odom_running_;  
}

// Collision Check
void ExecutionCheckerService::collision_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
    if(msg->states.size() > 0)
    {
        last_msg_received_collision_ = this->get_clock()->now();
    }
}

void ExecutionCheckerService::collision_service_callback(
    const std_srvs::srv::SetBool_Request::SharedPtr request,
    const std_srvs::srv::SetBool_Response::SharedPtr response)
{
    collision_time_difference = (this->get_clock()->now().seconds() - last_msg_received_collision_.seconds());

    if (collision_time_difference < MAX_COLLISION_LOOKBACK_TIME)
    {
        collision_detected_ = true;
    }
    else
    {
        if(debug_callback)
            RCLCPP_INFO(this->get_logger(), "No collisions detected");
        collision_detected_ = false;
    }
    response->message = "";
    response->success = collision_detected_;
    
}

void ExecutionCheckerService::orientation_checker_service_callback(
    const std_srvs::srv::SetBool_Request::SharedPtr request,
    const std_srvs::srv::SetBool_Response::SharedPtr response)
{
    // if(debug_orientation)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Orientation Checker says all 4 wheels on the ground: ");
    //     RCLCPP_INFO(this->get_logger(), all_wheels_on_the_ground ? "true" : "false");
        
    // }
    response->message = "";
    response->success = all_wheels_on_the_ground;
    
    if(debug_orientation)
    {
        RCLCPP_INFO(this->get_logger(), "Orientation Checker Service Response says all 4 wheels on the ground: ");
        RCLCPP_INFO(this->get_logger(), response->success ? "true" : "false");
        
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("ExecutionCheckerService"), "ExecutionCheckerService now active.");

  rclcpp::spin(std::make_shared<ExecutionCheckerService>("execution_checker_service"));

  rclcpp::shutdown();

  return 0;
}
