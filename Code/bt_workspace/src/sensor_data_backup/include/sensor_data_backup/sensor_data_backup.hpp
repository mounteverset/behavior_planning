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
#include "std_srvs/srv/empty.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_msgs/srv/get_last_goal.hpp"
#include "bt_msgs/srv/get_pose.hpp"
#include "bt_msgs/srv/get_last_map.hpp"
#include "bt_msgs/srv/send_updated_map.hpp"

class SensorDataBackup : public rclcpp::Node
{
    public:
        SensorDataBackup(const std::string & node_name);

        ~SensorDataBackup() = default;


    private:
        bool debug;
        
        void cmd_vel_service_callback(
                const bt_msgs::srv::GetTwistArray_Request::SharedPtr request,
                const bt_msgs::srv::GetTwistArray_Response::SharedPtr response);

        void pub_last_goal_service_callback(
            const std_srvs::srv::Empty_Request::SharedPtr request,
            const std_srvs::srv::Empty_Response::SharedPtr response);

        void get_last_goal_service_callback(
            const bt_msgs::srv::GetLastGoal_Request::SharedPtr request,
            const bt_msgs::srv::GetLastGoal_Response::SharedPtr response);
        
        void get_last_pose_service_callback(
            const bt_msgs::srv::GetPose_Request::SharedPtr request,
            const bt_msgs::srv::GetPose_Response::SharedPtr response);        

        void get_collision_pose_service_callback(
            const bt_msgs::srv::GetPose_Request::SharedPtr request,
            const bt_msgs::srv::GetPose_Response::SharedPtr response);

        void save_collision_pose_service_callback(
            const std_srvs::srv::Empty_Request::SharedPtr request,
            const std_srvs::srv::Empty_Response::SharedPtr response);

        void get_last_map_service_callback(
            const bt_msgs::srv::GetLastMap_Request::SharedPtr request,
            const bt_msgs::srv::GetLastMap_Response::SharedPtr response);

        void send_updated_map_callback(
            const bt_msgs::srv::SendUpdatedMap_Request::SharedPtr request,
            const bt_msgs::srv::SendUpdatedMap_Response::SharedPtr response);

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void pose_update_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        //void queue_to_vector(std::queue<geometry_msgs::msg::Twist>& queue, std::vector<geometry_msgs::msg::Twist> twist_array);

        // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar;
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap;

        rclcpp::Service<bt_msgs::srv::GetTwistArray>::SharedPtr service_cmd_vel_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_pub_last_goal_;

        rclcpp::Service<bt_msgs::srv::GetLastGoal>::SharedPtr service_get_last_goal_;
        rclcpp::Service<bt_msgs::srv::GetPose>::SharedPtr service_get_last_pose_; 
        rclcpp::Service<bt_msgs::srv::GetPose>::SharedPtr service_get_collision_pose_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_save_collision_pose_;
        rclcpp::Service<bt_msgs::srv::GetLastMap>::SharedPtr service_get_last_map_;

        rclcpp::Service<bt_msgs::srv::SendUpdatedMap>::SharedPtr service_send_updated_map_;

        // std::vector<sensor_msgs::msg::LaserScan> vector_laser;
        // std::vector<nav_msgs::msg::Odometry> vector_odom;
        // std::vector<sensor_msgs::msg::Imu> vector_imu;
        nav_msgs::msg::OccupancyGrid last_map;
        std::vector<nav_msgs::msg::OccupancyGrid> vector_map;
        geometry_msgs::msg::PoseStamped last_goal;
        std::vector<geometry_msgs::msg::Twist> vector_cmd_vel_;
        std::vector<geometry_msgs::msg::PoseStamped> vector_poses_;
        geometry_msgs::msg::Pose collision_pose_;

        rclcpp::Time reset_time;

};