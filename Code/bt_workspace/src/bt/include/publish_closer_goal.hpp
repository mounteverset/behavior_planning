#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "bt_msgs/srv/get_last_goal.hpp"
#include "bt_msgs/srv/get_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

/**
 * @brief Publish a new /goal_pose which is on the vector between robot pose and old goal pose closer to the robot.
 * 
 */
class PublishCloserGoal : public BT::SyncActionNode
{
public:

    PublishCloserGoal(const std::string& name) : BT::SyncActionNode(name, {})
    {   
        node_= rclcpp::Node::make_shared("publish_closer_goal");
        debug = false;

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor PublishCloserGoal");

        goal_service_client_ = node_->create_client<bt_msgs::srv::GetLastGoal>("get_last_goal_service");
        pose_service_client_ = node_->create_client<bt_msgs::srv::GetPose>("get_last_pose_service");

        goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 1);


        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor PublishCloserGoal finished.");
    }


    bool last_goal_service_call(geometry_msgs::msg::Pose& pose)
    {
        while (!goal_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get LastGoal Service not available, waiting again...");
        }

        auto request = std::make_shared<bt_msgs::srv::GetLastGoal::Request>();

        auto result = goal_service_client_->async_send_request(request);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sending Service Request.");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            pose = result.get()->last_goal_pose;
            return true;
        }
        else
        {
            return false;
        }

        return false;

    }

    bool last_pose_service_call(geometry_msgs::msg::Pose& pose)
    {
        while (!pose_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetLastPose Service not available, waiting again...");
        }

        auto request = std::make_shared<bt_msgs::srv::GetPose::Request>();

        auto result = pose_service_client_->async_send_request(request);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sending Service Request.");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            pose = result.get()->last_robot_pose;
            return true;
        }
        else
        {
            return false;
        }

        return false;

    }

    /**
     * @brief Create the vector from robot_pose to goal_pose
     * For that subtract robot_pose from goal_pose
     * 
     * @param goal_pose 
     * @param robot_pose 
     * @return geometry_msgs::msg::Vector3 
     */
    geometry_msgs::msg::Vector3 calc_vector(const geometry_msgs::msg::Pose goal_pose, const geometry_msgs::msg::Pose robot_pose)
    {
        geometry_msgs::msg::Vector3 vec;
        vec.x = goal_pose.position.x - robot_pose.position.x;
        vec.y = goal_pose.position.y - robot_pose.position.y;
        vec.z = 0.0;
        return vec;
    }

    float calc_distance(geometry_msgs::msg::Point point1, geometry_msgs::msg::Point point2)
    {
        return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));       
    }

    geometry_msgs::msg::Point calc_new_point(const geometry_msgs::msg::Vector3 vec, geometry_msgs::msg::Pose robot_pose, float factor)
    {
        geometry_msgs::msg::Point new_goal;
        new_goal.x = robot_pose.position.x + (factor * vec.x);
        new_goal.y = robot_pose.position.y + (factor * vec.y);
        new_goal.z = 0.0;

        return new_goal;
    } 

    geometry_msgs::msg::Point find_new_goal(const geometry_msgs::msg::Pose goal_pose, const geometry_msgs::msg::Pose robot_pose)
    {
        geometry_msgs::msg::Vector3 vec = calc_vector(goal_pose, robot_pose);
        geometry_msgs::msg::Point p1;
        float distance = 0.0;
        float factor = 0.99;

        while (distance < 0.1)
        {
            p1 = calc_new_point(vec, robot_pose, factor);
            distance = calc_distance(p1, goal_pose.position);
            factor -= 0.02;
        }

        return p1;
    }

    BT::NodeStatus tick() override
    {   
        geometry_msgs::msg::Pose goal_pose = geometry_msgs::msg::Pose();
        geometry_msgs::msg::Pose robot_pose = geometry_msgs::msg::Pose();


        if(!last_goal_service_call(goal_pose))
            return BT::NodeStatus::FAILURE;
        
        if(!last_pose_service_call(robot_pose))
            return BT::NodeStatus::FAILURE;

        geometry_msgs::msg::Point new_goal_position = find_new_goal(goal_pose, robot_pose);

        geometry_msgs::msg::PoseStamped new_goal;
        new_goal.pose.position = new_goal_position;
        new_goal.pose.orientation = goal_pose.orientation;
        new_goal.header.frame_id = "map";
        new_goal.header.stamp = node_->get_clock()->now();
        goal_publisher_->publish(new_goal);

        return BT::NodeStatus::SUCCESS;
    }


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<bt_msgs::srv::GetLastGoal>::SharedPtr goal_service_client_;
    rclcpp::Client<bt_msgs::srv::GetPose>::SharedPtr pose_service_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    // std::shared_ptr<std_srvs::srv::Empty::Request> request_;
    bool debug;

};