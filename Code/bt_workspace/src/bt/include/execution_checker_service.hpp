#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define MAX_ALLOWED_TIME_DIFFERENCE 1.0

class ExecutionCheckerService : public rclcpp::Node
{
    public:
        explicit ExecutionCheckerService (const std::string & node_name);

        virtual ~ExecutionCheckerService();

    private:
        void LidarExecutionCheckServiceCallback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lidar_service_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

        rclcpp::Time last_msg_received_;

        bool is_lidar_running_;

        float time_difference;

        bool debug_callback;
        bool debug;
};