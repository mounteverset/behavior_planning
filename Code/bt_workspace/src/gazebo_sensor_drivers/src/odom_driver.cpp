#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/publisher_options.hpp"
#include "nav_msgs/msg/odometry.hpp"

// #include "lifecycle_msgs/msg/transition.hpp"

// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"

class OdomDriver : public rclcpp::Node
{
  public: 
    OdomDriver() : Node("odom_driver")
    {
      if(debug)
        RCLCPP_INFO(this->get_logger(), "Odom Driver active. Republishing messages.");

      rclcpp::QoS topics_qos = rclcpp::SystemDefaultsQoS();      
      topics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      // rclcpp::QoS qos = rclcpp::QoS::transient_local();
      pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", topics_qos);
      sub_ = this->create_subscription<nav_msgs::msg::Odometry>("gazebo_odom", 10, std::bind(&OdomDriver::callback, this, std::placeholders::_1));
      
      rclcpp::Parameter sim_time_param("use_sim_time", rclcpp::ParameterValue(true));
      this->set_parameter(sim_time_param);    

      //tf2_ros::TransformListener tf_listener (buffer);
    }

    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if(debug)
        RCLCPP_INFO(this->get_logger(), "%d", msg->header.stamp.sec);

      // tf = buffer.lookupTransform()

      pub_->publish(*msg);
    }

  private:

    bool debug = false;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    //tf2_ros::TransformListener tf_listener;
    // geometry_msgs::msg::TransformStamped tf;
    // tf2_ros::Buffer buffer;
    // tf2_ros::TransformListener tf_listener (tf2_ros::Buffer buffer);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<OdomDriver>());
  rclcpp::shutdown();
  return 0; 

}