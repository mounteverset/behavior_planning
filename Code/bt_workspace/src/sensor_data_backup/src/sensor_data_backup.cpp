#include "sensor_data_backup.hpp"


void SensorDataBackup::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
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
    response->cmd_vel_array = vector_cmd_vel;
}



// void SensorDataBackup::queue_to_vector(std::queue<geometry_msgs::msg::Twist>& queue, std::vector<geometry_msgs::msg::Twist> twist_array)

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SensorDataBackup>("sensor_data_backup"));
}
 