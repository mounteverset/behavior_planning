#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_msgs/srv/pub_cmd_vel.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#define MIN_DRIVING_SPEED 0.05

class CmdVelDecisionGate : public rclcpp::Node 
{
    public:
        CmdVelDecisionGate(const std::string & node_name, const rclcpp::NodeOptions & options);

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        void pub_cmd_vel_service_callback(
                    const bt_msgs::srv::PubCmdVel_Request::SharedPtr request,
                    const bt_msgs::srv::PubCmdVel_Response::SharedPtr response);

        void decel_service_callback(
            const std_srvs::srv::SetBool_Request::SharedPtr request,
            const std_srvs::srv::SetBool_Response::SharedPtr response);

        void pub_complete_stop();

        void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void cmd_vel_bt_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void add_node_to_executor(rclcpp::Node::SharedPtr node_ptr);

        void publish_bt_cmd_vel();

        void spin_node();
    
    private:

        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_bt_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

        // rclcpp::Service<bt_msgs::srv::PubCmdVel>::SharedPtr service_pub_cmd_vel_;


        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr decel_service;

        rclcpp::executors::SingleThreadedExecutor exec_;
        
        // double publish_duration;
        // bool service_called; 

        bool decel_override_flag;
        // bool bt_override_flag;
        
        // geometry_msgs::msg::Twist bt_twist_msg;

        geometry_msgs::msg::Twist last_nav_msg;

        // rclcpp::Time time_when_last_nav_msgs_received;
        // rclcpp::Time time_when_service_call_received;

        bool debug;
};
