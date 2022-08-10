#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bt_msgs/srv/pub_cmd_vel.hpp"
#include "std_srvs/srv/set_bool.hpp"


class CmdVelDecisionGate : public rclcpp::Node //, public std::enable_shared_from_this<CmdVelDecisionGate>()
{
  public:
  CmdVelDecisionGate(const std::string & node_name) : Node(node_name)
  {
    sub_nav_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 10, std::bind(&CmdVelDecisionGate::cmd_vel_nav_callback, this, std::placeholders::_1));
    sub_bt_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_bt", 10, std::bind(&CmdVelDecisionGate::cmd_vel_bt_callback, this, std::placeholders::_1));
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    service_pub_cmd_vel_ = this->create_service<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service", std::bind(&CmdVelDecisionGate::pub_cmd_vel_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    //enable_shared_from_this(this);

    // exec_.add_node()

    service_called = false;
    debug = true;

    if(debug)
    RCLCPP_INFO(this->get_logger(), "Finished init.");
  }

/**
 * @brief 
 * 
 * @param request 
 * @param response 
 */
  void pub_cmd_vel_service_callback(
            const bt_msgs::srv::PubCmdVel_Request::SharedPtr request,
            const bt_msgs::srv::PubCmdVel_Response::SharedPtr response)
  {
    if(debug)
    RCLCPP_INFO(this->get_logger(), "Received service call.");
    bt_twist_msg = request->cmd_vel;
    publish_duration = request->time_in_seconds;
    time_when_service_call_received = this->get_clock()->now();
    service_called = true;
    response->success = true;
  }



/**
 * @brief 
 * Check if the service was called before republishing.
 * If the service was called, dont republish the Twist msg from Navigation.
 * Otherwise we simply republish the message.
 * 
 * @param msg 
 */
  void cmd_vel_nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  { 
    if (!service_called)
    {
      if(debug)
        RCLCPP_INFO(this->get_logger(), "Republishing Navigation cmd_vel.");
      pub_cmd_vel_->publish(*msg);
    }
  } 

/**
 * @brief 
 * 
 * @param msg 
 */
  void cmd_vel_bt_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {

  }

void add_node_to_executor(rclcpp::Node::SharedPtr node_ptr)
{
  if(debug)
    RCLCPP_INFO(this->get_logger(), "Node added to executor.");
  exec_.add_node(node_ptr);
}
/**
 * @brief 
 * Spinning the node to wait for cmd_vel_nav msgs.
 * Checking if the bt called the pub service.
 * If it was called, publish the twist msg from the service call as long as the 
 * service requested it.
 * 
 */
  void spin_node()
  {
    while (rclcpp::ok())
    {
      // rclcpp::spin_some(this->shared_from_this());
      
      exec_.spin_once();
      // exec_.spin();
      if(service_called && (this->get_clock()->now().seconds() - time_when_service_call_received.seconds() < publish_duration))
      {
        pub_cmd_vel_->publish(bt_twist_msg);
      }
      else
      {
        service_called = false;
        bt_twist_msg = geometry_msgs::msg::Twist();
        publish_duration = 0;
      }

    }
  }


  private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_bt_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_set_override;
  rclcpp::Service<bt_msgs::srv::PubCmdVel>::SharedPtr service_pub_cmd_vel_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  double publish_duration;
  bool service_called; 
  geometry_msgs::msg::Twist bt_twist_msg;

  rclcpp::Time time_when_service_call_received;

  bool debug;
};


 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_handle = std::make_shared<CmdVelDecisionGate>("cmd_vel_decision_gate");
  node_handle->add_node_to_executor(node_handle);
  node_handle->spin_node();

  return 0;
}
