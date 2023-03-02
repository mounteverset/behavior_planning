#include "sensor_data_backup.hpp"

SensorDataBackup::SensorDataBackup(const std::string & node_name) : Node(node_name)
{
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 10, std::bind(&SensorDataBackup::cmd_vel_callback, this, std::placeholders::_1));
    service_cmd_vel_ = this->create_service<bt_msgs::srv::GetTwistArray>("get_last_cmd_velocities", std::bind(&SensorDataBackup::cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_updated", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&SensorDataBackup::goal_callback, this, std::placeholders::_1));
    service_pub_last_goal_ = this->create_service<std_srvs::srv::Empty>("pub_last_goal_service", std::bind(&SensorDataBackup::pub_last_goal_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), std::bind(&SensorDataBackup::map_callback, this, std::placeholders::_1));
    sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("robot_pose", 1, std::bind(&SensorDataBackup::pose_update_callback, this, std::placeholders::_1));

    sub_global_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap", 1, std::bind(&SensorDataBackup::global_costmap_callback, this, std::placeholders::_1));
    service_get_collision_pose_ = this->create_service<bt_msgs::srv::GetPose>("get_collision_pose_service", std::bind(&SensorDataBackup::get_collision_pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_save_collision_pose_ = this->create_service<std_srvs::srv::Empty>("save_collision_pose_service", std::bind(&SensorDataBackup::save_collision_pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_last_map_ = this->create_service<bt_msgs::srv::GetLastMap>("get_last_map_service", std::bind(&SensorDataBackup::get_last_map_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    service_get_last_goal_ = this->create_service<bt_msgs::srv::GetLastGoal>("get_last_goal_service", std::bind(&SensorDataBackup::get_last_goal_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_last_pose_ = this->create_service<bt_msgs::srv::GetPose>("get_last_pose_service", std::bind(&SensorDataBackup::get_last_pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    // service_get_collsion_pose_ = this->create_service<bt_msgs::srv::GetPose>("get_collision_pose_service", std::bind(&SensorDataBackup::get_collision_pose_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    service_send_updated_map_ = this->create_service<bt_msgs::srv::SendUpdatedMap>("send_updated_map_service", std::bind(&SensorDataBackup::send_updated_map_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter("debug", false);

    debug = this->get_parameter("debug").as_bool();
}

rcl_interfaces::msg::SetParametersResult SensorDataBackup::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    if (!parameters.empty()) {
        result.successful = false;
        result.reason = "unknown parameter";
        return result;
    }

    for (auto & parameter : parameters) {
        if (parameter.get_name() == "debug") 
        {
            debug = parameter.as_bool();
            result.successful = true;
            result.reason = "success";
        }
        else 
        {
            result.successful = false;
            result.reason = "unknown parameter";
        }
    }

    return result;
}

void SensorDataBackup::send_updated_map_callback(
    const bt_msgs::srv::SendUpdatedMap_Request::SharedPtr request,
    const bt_msgs::srv::SendUpdatedMap_Response::SharedPtr response)
{   
    request->placeholder;
    if(debug)
        RCLCPP_INFO(this->get_logger(), "Received a service call for saving the map.");
    last_map = request->updated_map;
    pub_map_->publish(last_map);

}

void SensorDataBackup::pose_update_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (debug)
        RCLCPP_INFO(this->get_logger(), "Received AMCL Pose");
        
    vector_poses_.push_back(*msg);

    if(vector_poses_.size() > 20)
    {
        vector_poses_.erase(vector_poses_.begin());
    }
}

void SensorDataBackup::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    last_map = *msg;
}

void SensorDataBackup::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (debug)
        RCLCPP_INFO(this->get_logger(), "Received a Goal Pose");
    last_goal = *msg;
}

void SensorDataBackup::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{   
    if (debug)
        RCLCPP_INFO(this->get_logger(), "Received Cmd Vel");
        
    vector_cmd_vel_.push_back(*msg);

    if(vector_cmd_vel_.size() > 60)
    {
        vector_cmd_vel_.erase(vector_cmd_vel_.begin());
    }
}

void SensorDataBackup::pub_last_goal_service_callback(
    const std_srvs::srv::Empty_Request::SharedPtr request,
    const std_srvs::srv::Empty_Response::SharedPtr response)
{
    // request->placeholder;
    if(last_goal != geometry_msgs::msg::PoseStamped())
    {
        if(debug)
            RCLCPP_INFO(this->get_logger(), "Publishing last saved goal.");
        pub_goal_->publish(last_goal);
    }
    else 
    {
        if(debug)
            RCLCPP_INFO(this->get_logger(), "No Goal saved to republish.");
    }
}

void SensorDataBackup::cmd_vel_service_callback(
    const bt_msgs::srv::GetTwistArray_Request::SharedPtr request,
    const bt_msgs::srv::GetTwistArray_Response::SharedPtr response)
{
    request->placeholder;
    // queue_to_vector(queue_cmd_vel, vec);
    if (debug)
        RCLCPP_INFO(this->get_logger(), "Length of saved cmd_vel array: %ld", vector_cmd_vel_.size());
    if (vector_cmd_vel_.size() == 0)
        response->cmd_vel_array = std::vector<geometry_msgs::msg::Twist>();
    else
        response->cmd_vel_array = vector_cmd_vel_;
}


// Collision Sequence Related

void SensorDataBackup::get_last_goal_service_callback(
            const bt_msgs::srv::GetLastGoal_Request::SharedPtr request,
            const bt_msgs::srv::GetLastGoal_Response::SharedPtr response)
{
    request->placeholder;
    response->last_goal_pose = this->last_goal.pose;
}
        
void SensorDataBackup::get_last_pose_service_callback(
    const bt_msgs::srv::GetPose_Request::SharedPtr request,
    const bt_msgs::srv::GetPose_Response::SharedPtr response)
{
    request->placeholder;
    response->last_robot_pose = this->vector_poses_.back().pose;
}

void SensorDataBackup::global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Length of Costmap Data Array: %d", msg->data.size());
    // RCLCPP_INFO(this->get_logger(), "Length of Costmap Data Array: %f", msg->data.size());
}

void SensorDataBackup::save_collision_pose_service_callback(
    const std_srvs::srv::Empty_Request::SharedPtr request,
    const std_srvs::srv::Empty_Response::SharedPtr response)
{
    if(debug)
    {   
        // request->placeholder;
        RCLCPP_INFO(this->get_logger(), "Saving Pose with this coordinates\n x: %f \n y: %f",                        
                                                vector_poses_.back().pose.position.x, 
                                                vector_poses_.back().pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Saving Pose at timestamp: %f", 
                                                (vector_poses_.back().header.stamp.sec + vector_poses_.back().header.stamp.nanosec * pow(10, -9)));
    }
    collision_pose_ = this->vector_poses_.back().pose;
}

void SensorDataBackup::get_collision_pose_service_callback(
    const bt_msgs::srv::GetPose_Request::SharedPtr request,
    const bt_msgs::srv::GetPose_Response::SharedPtr response)
{   
    request->placeholder;
    response->last_robot_pose = collision_pose_; 
}

void SensorDataBackup::get_last_map_service_callback(
    const bt_msgs::srv::GetLastMap_Request::SharedPtr request,
    const bt_msgs::srv::GetLastMap_Response::SharedPtr response)
{
    if(debug)
    {
        RCLCPP_INFO(this->get_logger(), "Memory adress of request: %p", request.get());
    }
    response->map = last_map;
}
// void SensorDataBackup::queue_to_vector(std::queue<geometry_msgs::msg::Twist>& queue, std::vector<geometry_msgs::msg::Twist> twist_array)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SensorDataBackup>("sensor_data_backup"));
}
 