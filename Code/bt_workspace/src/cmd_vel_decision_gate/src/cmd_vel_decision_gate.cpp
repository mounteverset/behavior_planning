#include "cmd_vel_decision_gate.hpp"

CmdVelDecisionGate::CmdVelDecisionGate(
  const std::string & node_name, 
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions().allow_undeclared_parameters(true)) : Node(node_name, options)
{
    sub_nav_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 1, std::bind(&CmdVelDecisionGate::cmd_vel_nav_callback, this, std::placeholders::_1));
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    service_pub_cmd_vel_ = this->create_service<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service", std::bind(&CmdVelDecisionGate::pub_cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));


    service_called = false;
    debug = true;

    this->declare_parameter("bt_override", false);
    
    // What is the diff between these two lines?
    bt_override_flag = this->get_parameter("bt_override").as_bool();

    this->get_parameter("bt_override", bt_override_flag);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CmdVelDecisionGate::parametersCallback, this, std::placeholders::_1));

    

    if(debug)
        RCLCPP_INFO(this->get_logger(), "Finished init.");
}

/**
 * @brief 
 * 
 * @param request 
 * @param response 
 */
void CmdVelDecisionGate::pub_cmd_vel_service_callback(
          const bt_msgs::srv::PubCmdVel_Request::SharedPtr request,
          const bt_msgs::srv::PubCmdVel_Response::SharedPtr response)
{
  service_called = true;

//   if(debug)
//   {
//     RCLCPP_INFO(this->get_logger(), "Received service call.");
//     RCLCPP_INFO(this->get_logger(), "Linear.x: %f", request->cmd_vel.linear.x);
//     RCLCPP_INFO(this->get_logger(), "Angular.z: %f", request->cmd_vel.angular.z);
//     RCLCPP_INFO(this->get_logger(), "Publish Time in Seconds: %f", request->time_in_seconds);
//   }

  bt_twist_msg.linear.x = request->cmd_vel.linear.x;
  bt_twist_msg.linear.y = request->cmd_vel.linear.y;
  bt_twist_msg.angular.z = request->cmd_vel.angular.z;

  publish_duration = request->time_in_seconds;
  time_when_service_call_received = this->get_clock()->now();
  response->success = true;

  // if((bt_twist_msg.linear.x = 0.0) && (bt_twist_msg.angular.z = 0.0))
  //   pub_complete_stop();
  //   if(debug)
  //     RCLCPP_INFO(this->get_logger(), "Pub complete stop.");
}

/**
 * @brief The bt_override parameter will be changed from BT nodes, when they are publishing for extended periods of time.
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

    if(param.get_name() == "bt_override")
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        bt_override_flag = param.as_bool();

        if(debug)
          RCLCPP_INFO(this->get_logger(), "Changed bt_override.");
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
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

/**
 * @brief 
 * Check if the service was called before republishing.
 * If the service was called, dont republish the Twist msg from Navigation.
 * Otherwise we simply republish the message.
 * 
 * @param msg 
 */
void CmdVelDecisionGate::cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{ 
  time_when_last_nav_msgs_received = this->get_clock()->now();

  rclcpp::Parameter param = this->get_parameter("bt_override");

  bt_override_flag = param.as_bool();


  if(debug)
  {
    RCLCPP_INFO(this->get_logger(), "bt_override value param: %s", param.value_to_string().c_str());
    RCLCPP_INFO(this->get_logger(), "bt_override_flag variable: %s", bt_override_flag ? "true" : "false");

  }

  if (!service_called && !bt_override_flag)
  {
    if(debug)
      RCLCPP_INFO(this->get_logger(), "Republishing Navigation cmd_vel.");

    pub_cmd_vel_->publish(*msg);
    last_nav_msg = *msg;
  }
} 

void CmdVelDecisionGate::add_node_to_executor(rclcpp::Node::SharedPtr node_ptr)
{
  if(debug)
    RCLCPP_INFO(this->get_logger(), "Node added to executor.");
  exec_.add_node(node_ptr);
}

/**
 * @brief Blocking the spinning of the node and publishing BT cmd_vel for as long as the pub_cmd_vel_service call specified it
 * The publish duration is used my the decel_to_min_driving_speed node of the BT
 * The reverse_cmd_vel node should only use very small publish duration as the publish duration is handled in the BT by calling this nodes service with the new cmd_vel intermittently.
 * This is so that the node can go back to spinning and is ready for the next service call.
 * While spinning the nav_cmd_vel are getting ignored as the bt_override param was set by the reverse_cmd_vel node.
 */
void CmdVelDecisionGate::publish_bt_cmd_vel()
{
  if(debug)
    RCLCPP_INFO(this->get_logger(), "Publishing BT Cmd Vel.");
  
  if(last_nav_msg.linear.x != 0.0 || last_nav_msg.angular.z != 0.0)
  {     
    while (publish_duration > 0) 
    {
      pub_cmd_vel_->publish(bt_twist_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(30));
      publish_duration -= 0.03;
    }
  }
  else
  {
    pub_complete_stop();
  }

  service_called = false;
}

/**
 * @brief 
 * Spinning the node to wait for cmd_vel_nav msgs.
 * Checking if the bt called the pub service.
 * If it was called, publish the twist msg from the service call as long as the 
 * service requested it. 
 * 
 */
void CmdVelDecisionGate::spin_node()
{
  while (rclcpp::ok())
  {
    // Check for service calls and published msgs
    exec_.spin_once();

    if(service_called)
    {      
      publish_bt_cmd_vel();
    }    
  }
}

/*
    // if(service_called 
    //   && (this->get_clock()->now().seconds() - time_when_service_call_received.seconds() < publish_duration) 
    //   && (last_nav_msg.linear.x > 0.0))
    // {
    //   if(debug)
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Publishing received service_call.");
    //     RCLCPP_INFO(this->get_logger(), "Time Difference Service Call to now: ");
    //     RCLCPP_INFO(this->get_logger(), std::to_string(this->get_clock()->now().seconds() - time_when_service_call_received.seconds()));
    //   }
    //   pub_cmd_vel_->publish(bt_twist_msg);
    // }
    // else
    // { 
    //   // Auto Stop Behaviour if we published min_speed but did not get another cmd_vel from nav2 normally
    //   // Otherwise the last published msg of BT would be min driving speed, hence we have to publish a stop cmd if no new msg from nav come in
    //   if ((this->get_clock()->now().seconds() - time_when_last_nav_msgs_received.seconds()) > 1.0)
    //     pub_complete_stop();
      
    //   // Reset the service call variables
    //   service_called = false;
    //   bt_twist_msg = geometry_msgs::msg::Twist();       
    //   publish_duration = 0;
    // }

  }
}
*/

 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_handle = std::make_shared<CmdVelDecisionGate>("cmd_vel_decision_gate");
  node_handle->add_node_to_executor(node_handle);
  node_handle->spin_node();

  return 0;
}
