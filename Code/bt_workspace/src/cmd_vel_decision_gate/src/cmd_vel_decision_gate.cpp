#include "cmd_vel_decision_gate.hpp"

CmdVelDecisionGate::CmdVelDecisionGate(
  const std::string & node_name, 
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions().allow_undeclared_parameters(true)) : Node(node_name, options)
{
    sub_nav_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 1, std::bind(&CmdVelDecisionGate::cmd_vel_nav_callback, this, std::placeholders::_1));
    sub_bt_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_bt", 1, std::bind(&CmdVelDecisionGate::cmd_vel_bt_callback, this, std::placeholders::_1));
    
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // service_pub_cmd_vel_ = this->create_service<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service", std::bind(&CmdVelDecisionGate::pub_cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    decel_service = this->create_service<std_srvs::srv::SetBool>("decel_min_speed_service", std::bind(&CmdVelDecisionGate::decel_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // service_called = false;

    this->declare_parameter("debug", false);
    
    debug = this->get_parameter("debug").as_bool();

    // this->declare_parameter("bt_override", false);

    this->declare_parameter("decel_flag", false);

    decel_override_flag = this->get_parameter("decel_flag").as_bool();
    
    // What is the diff between these two lines?
    // bt_override_flag = this->get_parameter("bt_override").as_bool();

    // this->get_parameter("bt_override", bt_override_flag);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CmdVelDecisionGate::parametersCallback, this, std::placeholders::_1));
    
    if(debug)
        RCLCPP_INFO(this->get_logger(), "Finished init.");
}

/**
 * @brief Service to set the deceleration during restart of the sensor nodes
 * 
 * @param request 
 * @param response 
 */
void CmdVelDecisionGate::decel_service_callback(
        const std_srvs::srv::SetBool_Request::SharedPtr request,
        const std_srvs::srv::SetBool_Response::SharedPtr response)
{
    rclcpp::Parameter param ("decel_flag", request->data);
    this->set_parameter(param);
    decel_override_flag = request->data;
    response->message = "";
    response->success = true;
}

/**
 * @brief 
 * 
 * @param parameters 
 * @return rcl_interfaces::msg::SetParametersResult 
 */
rcl_interfaces::msg::SetParametersResult CmdVelDecisionGate::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    if(debug)
    RCLCPP_INFO(this->get_logger(), "Parameter Callback called.");

    for (auto &param : parameters)
    {

        RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());

        if(debug)
            RCLCPP_INFO(this->get_logger(), "Parameter Name: %s", param.get_name().c_str());

        // this->

        // if(param.get_name() == "bt_override")
        // {
        //   if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
        //   {
        //     bt_override_flag = param.as_bool();

        //     if(debug)
        //       RCLCPP_INFO(this->get_logger(), "Changed bt_override.");
        //   }
        // }
        if(param.get_name() == "decel_flag")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                decel_override_flag = param.as_bool();
                if(debug)
                    RCLCPP_INFO(this->get_logger(), "Changed decel_flag.");
            }
        }
    }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

/**
 * @brief Republishes the incoming navigation cmd_vel
 * Changes to speed if the decel_flag param is set to true 
 * 
 * @param msg 
 */
void CmdVelDecisionGate::cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{ 
    // time_when_last_nav_msgs_received = this->get_clock()->now();

    if(decel_override_flag)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        if (msg->linear.x != 0.0)
        {
            if(msg->linear.x > 0.0)
            {
                twist_msg.linear.x = MIN_DRIVING_SPEED; // 0.05
            }
            else
            {
                twist_msg.linear.x = -MIN_DRIVING_SPEED;
            }
        }
        else
        {
            twist_msg.linear.x = 0.0;
        }
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = twist_msg.angular.z * MIN_DRIVING_SPEED / msg->linear.x; // decrease the turning of the robot by the same factor as the linear speed was decreased
        pub_cmd_vel_->publish(twist_msg);
    }
    else if (!decel_override_flag)
    {
        if(debug)
        RCLCPP_INFO(this->get_logger(), "Republishing Navigation cmd_vel.");

        pub_cmd_vel_->publish(*msg);
        last_nav_msg = *msg;
    }
} 

void CmdVelDecisionGate::cmd_vel_bt_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    pub_cmd_vel_->publish(*msg);
}

void CmdVelDecisionGate::add_node_to_executor(rclcpp::Node::SharedPtr node_ptr)
{
    if(debug)
        RCLCPP_INFO(this->get_logger(), "Node added to executor.");
    exec_.add_node(node_ptr);
}

void CmdVelDecisionGate::pub_complete_stop()
{
  if(debug)
    RCLCPP_INFO(this->get_logger(), "Pub complete stop.");

  geometry_msgs::msg::Twist stop_msg = geometry_msgs::msg::Twist();
  stop_msg.linear.x = 0.0;
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;
  stop_msg.angular.x = 0.0;
  stop_msg.angular.y = 0.0;
  stop_msg.angular.z = 0.0;

  pub_cmd_vel_->publish(stop_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    while(rclcpp::ok())
    {
        rclcpp::spin(std::make_shared<CmdVelDecisionGate>("cmd_vel_decision_gate"));
    }

    return 0;
}