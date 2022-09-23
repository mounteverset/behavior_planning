#include "scenario_1.hpp"

BehaviorTester::BehaviorTester(std::string &name) : rclcpp::Node(name)
    {
        // sdf_file_path = (os.path.join(get_package_share_directory(package_name), 'urdf', 'radu_gazebo_compiled.urdf')),

        this->declare_parameter("spawn_pose_x", 0.0);
        this->declare_parameter("spawn_pose_y", 0.0);
        this->declare_parameter("spawn_pose_yaw", 0.0);

        this->declare_parameter("goal_pose_x", 1.0);
        this->declare_parameter("goal_pose_y", 1.0);
        this->declare_parameter("goal_pose_yaw", 0.0);

        this->get_parameter("spawn_pose_x", robot_spawn_pose_.position.x);
        this->get_parameter("spawn_pose_y", robot_spawn_pose_.position.y);
        this->get_parameter("spawn_pose_yaw", robot_spawn_yaw_);
        tf2::Quaternion quat_robot;
        quat_robot.setRPY(0.0, 0.0, robot_spawn_yaw_);
        robot_spawn_pose_.orientation.w = quat_robot.getW();
        robot_spawn_pose_.orientation.x = quat_robot.getX();
        robot_spawn_pose_.orientation.y = quat_robot.getY();
        robot_spawn_pose_.orientation.z = quat_robot.getZ();

        this->get_parameter("goal_pose_x", goal_pose_.position.x);
        this->get_parameter("goal_pose_y", goal_pose_.position.y);
        this->get_parameter("goal_pose_yaw", goal_yaw_);
        tf2::Quaternion quat_goal;
        quat_goal.setRPY(0.0, 0.0, goal_yaw_);
        goal_pose_.orientation.w = quat_goal.getW();
        goal_pose_.orientation.x = quat_goal.getX();
        goal_pose_.orientation.y = quat_goal.getY();
        goal_pose_.orientation.z = quat_goal.getZ();

        spawn_client_handle_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");

        request_->name = "turtlebot";
        request_->robot_namespace = "";
        request_->reference_frame = "world";
        request_->initial_pose = robot_spawn_pose_;
        // request_->xml = 
    }

void BehaviorTester::spawn_entity()
{
  while (!spawn_client_handle_->wait_for_service(std::chrono::seconds(1)))
  {
      if (!rclcpp::ok()) 
      {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Odom Execution Service not available, waiting again...");
  }



  auto result = spawn_client_handle_->async_send_request()
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world bt_tests package\n");
  return 0;
}
