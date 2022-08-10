#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/publisher_options.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class LidarDriver : public rclcpp::Node
{
  public: 
    LidarDriver() : Node("lidar_driver")
    {
      if(debug)
        RCLCPP_INFO(this->get_logger(), "Lidar Driver active. Republishing messages.");

      rclcpp::QoS topics_qos = rclcpp::SystemDefaultsQoS();      
      topics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

      // rclcpp::QoS qos = rclcpp::QoS::transient_local();
      pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", topics_qos);
      sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("gazebo_scan", 10, std::bind(&LidarDriver::callback, this, std::placeholders::_1));
      
      rclcpp::Parameter sim_time_param("use_sim_time", rclcpp::ParameterValue(true));
      this->set_parameter(sim_time_param);    
    }

    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      pub_->publish(*msg);

    }

  private:

    bool debug = true;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LidarDriver>());
  rclcpp::shutdown();
  return 0;
}
