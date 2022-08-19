#include "sensor_data_backup.hpp"

SensorDataBackup::SensorDataBackup(const std::string & node_name) : Node(node_name)
{
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SensorDataBackup::cmd_vel_callback, this, std::placeholders::_1));
    service_cmd_vel = this->create_service<bt_msgs::srv::GetTwistArray>("get_last_cmd_velocities", std::bind(&SensorDataBackup::cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    bool debug = true;
}

void SensorDataBackup::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {   
        if (debug)
            RCLCPP_INFO(this->get_logger(), "Received Cmd Vel");
            
        vector_cmd_vel.push_back(*msg);

        if(vector_cmd_vel.size() > 60)
        {
            vector_cmd_vel.erase(vector_cmd_vel.begin());
        }
    }

void SensorDataBackup::cmd_vel_service_callback(
    const bt_msgs::srv::GetTwistArray_Request::SharedPtr request,
    const bt_msgs::srv::GetTwistArray_Response::SharedPtr response)
{
    // queue_to_vector(queue_cmd_vel, vec);
    if (debug)
        RCLCPP_INFO(this->get_logger(), "Length of saved cmd_vel array: %d", vector_cmd_vel.size());
    response->cmd_vel_array = vector_cmd_vel;
}



// void SensorDataBackup::queue_to_vector(std::queue<geometry_msgs::msg::Twist>& queue, std::vector<geometry_msgs::msg::Twist> twist_array)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SensorDataBackup>("sensor_data_backup"));
}
 