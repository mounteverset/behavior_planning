#include <queue>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "bt_msgs/srv/pub_cmd_vel.hpp"
#include "bt_msgs/srv/get_twist_array.hpp"

class SensorDataBackup : public rclcpp::Node
{
    public:
        SensorDataBackup(const std::string & node_name) : Node(node_name)
        {
            sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SensorDataBackup::cmd_vel_callback, this, std::placeholders::_1));
            service_cmd_vel = this->create_service<bt_msgs::srv::GetTwistArray>("get_last_cmd_vels", std::bind(&SensorDataBackup::cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));
        }

        ~SensorDataBackup() = default;


    private:

        void cmd_vel_service_callback(
                const bt_msgs::srv::GetTwistArray_Request::SharedPtr request,
                const bt_msgs::srv::GetTwistArray_Response::SharedPtr response);

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        //void queue_to_vector(std::queue<geometry_msgs::msg::Twist>& queue, std::vector<geometry_msgs::msg::Twist> twist_array);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;

        rclcpp::Service<bt_msgs::srv::GetTwistArray>::SharedPtr service_cmd_vel;
        

        std::vector<sensor_msgs::msg::LaserScan> vector_laser;
        std::vector<nav_msgs::msg::Odometry> vector_odom;
        std::vector<sensor_msgs::msg::Imu> vector_imu;
        std::vector<nav_msgs::msg::OccupancyGrid> vector_map;
        std::vector<geometry_msgs::msg::Twist> vector_cmd_vel;
        geometry_msgs::msg::PoseStamped last_goal;

        rclcpp::Time reset_time;

};