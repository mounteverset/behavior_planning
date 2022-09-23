#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class BehaviorTester : rclcpp::Node
{
  public:
    BehaviorTester(std::string &name);

    void spawn_entity();

  private:

  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_handle_;
  geometry_msgs::msg::Pose robot_spawn_pose_;
  float robot_spawn_yaw_;

  geometry_msgs::msg::Pose goal_pose_;
  float goal_yaw_;
  std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Request> request_;


};