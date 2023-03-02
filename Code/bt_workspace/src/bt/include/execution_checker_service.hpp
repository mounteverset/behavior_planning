#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "gazebo_msgs/msg/contact_state.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "bt_msgs/srv/get_distance.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "std_msgs/msg/bool.hpp"
// #include "tf2_conversion_helpers.hpp"
// #include "tf2_helpers/tf2_conversion_helpers.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#define MAX_ALLOWED_TIME_DIFFERENCE 1.0
#define MAX_COLLISION_LOOKBACK_TIME 1.0

class ExecutionCheckerService : public rclcpp::Node
{
    public:
        explicit ExecutionCheckerService (const std::string & node_name);

        virtual ~ExecutionCheckerService();

    private:

        //Dynamic parameter setting
        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & parameters);
        
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
        void collision_callback(const std_msgs::msg::Bool::SharedPtr msg);

        void rollover_callback(const std_msgs::msg::Bool::SharedPtr msg);

        void collision_service_callback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr collision_service_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr save_collision_pose_client;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collision_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_rollover_;
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

        void goal_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void pose_update_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        void calc_distance();

        rclcpp::Service<bt_msgs::srv::GetDistance>::SharedPtr distance_service_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;

        geometry_msgs::msg::PoseStamped goal = geometry_msgs::msg::PoseStamped();
        geometry_msgs::msg::PoseWithCovarianceStamped pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        double distance_to_goal;
        bool goal_received = false;
        bool pose_received = false;

        //Global Planner Related

        void plan_possible_service_callback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        void global_planner_goal_states_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);

        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr plan_possible_service_;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_global_planner_goal_states;
        bool is_global_plan_possible;




        // Debug Statement Related

        bool debug_callback;
        bool debug;
        bool debug_orientation;
        bool debug_distance;
};