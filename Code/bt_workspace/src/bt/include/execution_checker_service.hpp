#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "gazebo_msgs/msg/contact_state.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "bt_msgs/srv/get_distance.hpp"
// #include "tf2_conversion_helpers.hpp"
// #include "tf2_helpers/tf2_conversion_helpers.hpp"

#define MAX_ALLOWED_TIME_DIFFERENCE 1.0
#define MAX_COLLISION_LOOKBACK_TIME 1.0

class ExecutionCheckerService : public rclcpp::Node
{
    public:
        explicit ExecutionCheckerService (const std::string & node_name);

        virtual ~ExecutionCheckerService();

    private:
        // Lidar Related
        void LidarExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lidar_service_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
        rclcpp::Time last_msg_received_;
        bool is_lidar_running_;
        float time_difference;


        // IMU Related
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void ImuExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);
        
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imu_service_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Time last_msg_received_imu_;
        bool is_imu_running_;
        float time_difference_imu;


        // Odom Related
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void OdomExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr odom_service_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Time last_msg_received_odom_;
        bool is_odom_running_;
        float time_difference_odom;


        // Collision Related
        void collision_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg);

        void collision_service_callback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr collision_service_;
        rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr sub_collision_;
        rclcpp::Time last_msg_received_collision_;
        bool collision_detected_;
        float collision_time_difference;
        // float collision_max_lookback_time = ;

        // Orientation Related
        void orientation_checker_service_callback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr orientation_service_;    
        bool all_wheels_on_the_ground;

        // Battery Charge Related
        void goal_distance_service_callback(
            const bt_msgs::srv::GetDistance_Request::SharedPtr request,
            const bt_msgs::srv::GetDistance_Response::SharedPtr response);

        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void pose_update_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        void calc_distance();

        rclcpp::Service<bt_msgs::srv::GetDistance>::SharedPtr distance_service_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;

        geometry_msgs::msg::PoseStamped goal = geometry_msgs::msg::PoseStamped();
        geometry_msgs::msg::PoseWithCovarianceStamped pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        double distance_to_goal;
        bool goal_received = false;
        bool pose_received = false;

        // Debug Statement Related

        bool debug_callback;
        bool debug;
        bool debug_orientation;
};