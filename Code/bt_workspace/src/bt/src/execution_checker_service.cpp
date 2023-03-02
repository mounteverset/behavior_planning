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

    distance_service_ = this->create_service<bt_msgs::srv::GetDistance>(
        "goal_distance_service",
        std::bind(&ExecutionCheckerService::goal_distance_service_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    plan_possible_service_ = this->create_service<std_srvs::srv::SetBool>(
        "path_possible_check_service",
        std::bind(&ExecutionCheckerService::plan_possible_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    

    //Parameters for debugging 
    
    // Debug Boolean Flag
    this->declare_parameter("debug", false);
    this->declare_parameter("debug_orientation", false);
    this->declare_parameter("debug_distance", false);

    debug = this->get_parameter("debug").as_bool();
    debug_orientation = this->get_parameter("debug_orientation").as_bool();
    debug_distance = this->get_parameter("debug_distance").as_bool();

    // topics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(); // Transient Local Sub Settings
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&ExecutionCheckerService::lidar_callback, this, std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_broadcaster/imu", 10, std::bind(&ExecutionCheckerService::imu_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, std::bind(&ExecutionCheckerService::odom_callback, this, std::placeholders::_1));
    sub_collision_ = this->create_subscription<std_msgs::msg::Bool>("collision", 10, std::bind(&ExecutionCheckerService::collision_callback, this, std::placeholders::_1));
    // sub_rollover_ = this->create_subscription<std_msgs::msg::Bool>("rollover", 10, std::bind(&ExecutionCheckerService::collision_callback, this, std::placeholders::_1));
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>("plan", 1, std::bind(&ExecutionCheckerService::goal_callback, this, std::placeholders::_1));
    //sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1, std::bind(&ExecutionCheckerService::pose_update_callback, this, std::placeholders::_1));
    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&ExecutionCheckerService::pose_update_callback, this, std::placeholders::_1));
    sub_global_planner_goal_states = this->create_subscription<action_msgs::msg::GoalStatusArray>("compute_path_to_pose/_action/status", 10, std::bind(&ExecutionCheckerService::global_planner_goal_states_callback, this, std::placeholders::_1));

    save_collision_pose_client = this->create_client<std_srvs::srv::Empty>("save_collision_pose_service");

    // debug = false;
    // debug_orientation = true;
    // debug_distance = false;


    // Subtract 2 seconds from the current timestamp
    rclcpp::Time current = this->get_clock()->now();
    rclcpp::Duration two_secs = rclcpp::Duration((int32_t)2, (uint32_t)0);
    this->last_msg_received_collision_ = current - two_secs;

    //Set all execution variables to false
    this->collision_detected_ = false;
    this->all_wheels_on_the_ground = false;
    this->is_lidar_running_ = false;
    this->is_imu_running_ = false;
    this->is_odom_running_ = false;
}

ExecutionCheckerService::~ExecutionCheckerService() = default;

//Parameters Callback for dynamic reconfigure
rcl_interfaces::msg::SetParametersResult ExecutionCheckerService::paramCallback(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    if(debug)
        RCLCPP_INFO(this->get_logger(), "Received parameter change request");

    for (auto parameter : parameters)
    {
        if(debug)
        {
            RCLCPP_INFO(this->get_logger(), "%s", parameter.get_name().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", parameter.value_to_string().c_str());
            RCLCPP_INFO(this->get_logger(), "%s", parameter.get_type_name().c_str());
        }
        if(parameter.get_name() == "debug")
        {
            if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                debug = parameter.as_bool();
                result.successful =true;
                result.reason = "success"; 
            }
        }
        else if(parameter.get_name() == "debug_orientation")
        {
            if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                debug_orientation = parameter.as_bool();
                result.successful = true;
                result.reason = "success"; 
            }
        }
        else if(parameter.get_name() == "debug_distance")
        {
            if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                debug_distance = parameter.as_bool();
                result.successful = true;
                result.reason = "success";
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
        }
    }

    return result;
}

// Service and Topic Callbacks

// Lidar 
void ExecutionCheckerService::LidarExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response)
{   
    if (debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "Lidar callback");
        RCLCPP_INFO(this->get_logger(), "%s", request->data? "true" : "false");
    }

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

// Subsribes to Scan Topic and saves the timestamp
// The timestamp gets compared in the LidarExecutionServiceCallback with the current time
void ExecutionCheckerService::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{   
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "Received a Scan Message");
        RCLCPP_INFO(this->get_logger(), "%d", msg->header.stamp.sec);
        // RCLCPP_INFO(this->get_logger(), "%s", msg->header.stamp.toNSec());
        // RCLCPP_INFO(this->get_logger(), "last_msg_received at: %d", last_msg_received_.seconds());
        // RCLCPP_INFO(this->get_logger(), "msg.header.stamp at: %f", msg->header.stamp.sec);
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

    if(debug_orientation)
    {
        RCLCPP_INFO(this->get_logger(), "IMU Roll: %f", roll);
        RCLCPP_INFO(this->get_logger(), "IMU Pitch: %f", pitch);
    }

    if(debug_orientation)
    {
        RCLCPP_INFO(this->get_logger(), "Orientation Checker says all 4 wheels on the ground: ");
        RCLCPP_INFO(this->get_logger(), all_wheels_on_the_ground ? "true" : "false");
        
    }

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
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "%s", request->data ? "true" : "false");
    }
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
    if (msg->header.frame_id != "odom")
    {
        RCLCPP_WARN(this->get_logger(), "Received a faulty Odometry Message");
        return;
    }
    last_msg_received_odom_ = this->get_clock()->now();
}

void ExecutionCheckerService::OdomExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response)
{   
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "%s", request->data ? "true" : "false");
    }

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
void ExecutionCheckerService::collision_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data == true)
    {
        last_msg_received_collision_ = this->get_clock()->now();
    }
}


void ExecutionCheckerService::collision_service_callback(
    const std_srvs::srv::SetBool_Request::SharedPtr request,
    const std_srvs::srv::SetBool_Response::SharedPtr response)
{
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "%s", request->data ? "true" : "false");
    }
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

    if(collision_detected_)
    {
        auto response = save_collision_pose_client->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        
    }
    collision_detected_ = false;
}

void ExecutionCheckerService::rollover_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // if(msg->data == true)
    // {
    //     //last_msg_received_collision_ = this->get_clock()->now();
    //     this->all_wheels_on_the_ground = 
    // }
    this->all_wheels_on_the_ground = msg->data;
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
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "%s", request->data ? "true" : "false");
    }
    response->message = "";
    response->success = this->all_wheels_on_the_ground;
    
    if(debug_orientation)
    {
        RCLCPP_INFO(this->get_logger(), "Orientation Checker Service Response says all 4 wheels on the ground: ");
        RCLCPP_INFO(this->get_logger(), response->success ? "true" : "false");
        
    }
}


// Battery Sufficient Check
void ExecutionCheckerService::calc_distance()
{
    if(goal_received && pose_received)
    {
        distance_to_goal = sqrt(pow(goal.pose.position.x - pose.pose.pose.position.x, 2) + pow(goal.pose.position.y - pose.pose.pose.position.y, 2));
    }
    else
    {
        distance_to_goal = 0.0;
    }
}


void ExecutionCheckerService::goal_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if(debug_distance)
    {
        RCLCPP_INFO(this->get_logger(), "Received a goal!");   
    }

    goal = msg->poses.back();



    if(debug_distance)
    {
        RCLCPP_INFO(this->get_logger(), "goal pos.x: %f", goal.pose.position.x);  
        RCLCPP_INFO(this->get_logger(), "goal pos.y: %f", goal.pose.position.y);  
        RCLCPP_INFO(this->get_logger(), "msg pos.x: %f", msg->poses.back().pose.position.x); 
        RCLCPP_INFO(this->get_logger(), "msg pos.y: %f", msg->poses.back().pose.position.y); 
    }
    goal_received = true;
    calc_distance();
}


void ExecutionCheckerService::pose_update_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if(debug_distance)
    {
        RCLCPP_INFO(this->get_logger(), "Received a pose!");   
    }
    pose = *msg;

    if(debug_distance)
    {
        RCLCPP_INFO(this->get_logger(), "pose pos.x: %f", pose.pose.pose.position.x);  
        RCLCPP_INFO(this->get_logger(), "pose pos.y: %f", pose.pose.pose.position.y);  
        RCLCPP_INFO(this->get_logger(), "msg pos.x: %f", msg->pose.pose.position.x); 
        RCLCPP_INFO(this->get_logger(), "msg pos.y: %f", msg->pose.pose.position.y); 
    }
    pose_received = true;
    // calc_distance();
}


void ExecutionCheckerService::goal_distance_service_callback(
    const bt_msgs::srv::GetDistance_Request::SharedPtr request,
    const bt_msgs::srv::GetDistance_Response::SharedPtr response)
{
    request->placeholder;
    response->distance_in_meter = distance_to_goal;
}


// Global Path Possible Related
void ExecutionCheckerService::plan_possible_service_callback(   
    const std_srvs::srv::SetBool_Request::SharedPtr request,
    const std_srvs::srv::SetBool_Response::SharedPtr response)
{   
    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "%s", request->data ? "true" : "false");
    }
    response->message="";
    response->success = is_global_plan_possible;    
}   


void ExecutionCheckerService::global_planner_goal_states_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    auto state = msg->status_list.back();

    if(debug_callback)
    {
        RCLCPP_INFO(this->get_logger(), "Received a Goal Status Array");
        RCLCPP_INFO(this->get_logger(), "Time since last goal state added: %f", (this->get_clock()->now().seconds() - (state.goal_info.stamp.sec + state.goal_info.stamp.nanosec * pow(10, -9) )));
    }

    if ((this->get_clock()->now().seconds() - (state.goal_info.stamp.sec + state.goal_info.stamp.nanosec * pow(10, -9) )) > 0.1)
    {
        if (state.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
        {
            is_global_plan_possible = true;
        }
        // else if (state.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        // {
        //     is_global_plan_possible = false;
        // }
        else
        {
            is_global_plan_possible = false;
        }
    }
    else
    {
        if (state.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
        {
            is_global_plan_possible = true;
        }
        else if (state.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            is_global_plan_possible = true;
        }
        else
        {
            is_global_plan_possible = false;
        }
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
