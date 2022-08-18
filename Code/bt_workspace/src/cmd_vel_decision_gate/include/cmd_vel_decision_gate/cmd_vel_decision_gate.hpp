#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_msgs/srv/pub_cmd_vel.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class CmdVelDecisionGate : public rclcpp::Node 
{
    public:
        CmdVelDecisionGate(const std::string & node_name) : Node(node_name)
        {
            sub_nav_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 1, std::bind(&CmdVelDecisionGate::cmd_vel_nav_callback, this, std::placeholders::_1));
            pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            service_pub_cmd_vel_ = this->create_service<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service", std::bind(&CmdVelDecisionGate::pub_cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));


            service_called = false;
            debug = true;

            this->declare_parameter("bt_override", false);

            bt_override_flag = this->get_parameter("bt_override").as_bool();

            parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CmdVelDecisionGate::parametersCallback, this, std::placeholders::_1));

            if(debug)
                RCLCPP_INFO(this->get_logger(), "Finished init.");
        }

        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        void pub_cmd_vel_service_callback(
                    const bt_msgs::srv::PubCmdVel_Request::SharedPtr request,
                    const bt_msgs::srv::PubCmdVel_Response::SharedPtr response);

        void pub_complete_stop();

        void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void add_node_to_executor(rclcpp::Node::SharedPtr node_ptr);

        void publish_bt_cmd_vel();

        void spin_node();
    
    private:

        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

        rclcpp::Service<bt_msgs::srv::PubCmdVel>::SharedPtr service_pub_cmd_vel_;
        rclcpp::executors::SingleThreadedExecutor exec_;
        double publish_duration;
        bool service_called; 

        bool bt_override_flag;
        
        geometry_msgs::msg::Twist bt_twist_msg;

        geometry_msgs::msg::Twist last_nav_msg;

        rclcpp::Time time_when_last_nav_msgs_received;
        rclcpp::Time time_when_service_call_received;

        bool debug;
};
