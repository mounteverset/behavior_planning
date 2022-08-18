#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"

/**
 * @brief Returns Success if the robot is not in collision and free to move
 * 
 */
class CollisionCheck : public BT::ConditionNode
{
public:

    CollisionCheck(const std::string& name) : BT::ConditionNode(name, {})
    {   

        in_collision = false;
        debug = true;
        node_= rclcpp::Node::make_shared("collision_check");
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor CollisionCheck");
        service_client_ = node_->create_client<std_srvs::srv::SetBool>("collision_check_service");
        request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        request_->data = true;

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor CollisionCheck finished.");
    }

    BT::NodeStatus tick() override
    {
        auto result = service_client_->async_send_request(request_);

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            in_collision = result.get()->success;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }

        // getInput("is_lidar_running", is_lidar_running_);
        // (is_lidar_running_) ? is_lidar_running_ = true : is_lidar_running_ = false;

        if(debug)
        {
            RCLCPP_INFO(rclcpp::get_logger("collision_check"), "Robot is in collision: ");
            RCLCPP_INFO(rclcpp::get_logger("collision_check"), (in_collision) ? "true" : "false");
        }

        if (!in_collision)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else 
        {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
    std::shared_ptr<std_srvs::srv::SetBool::Request> request_;
    bool debug;
    bool in_collision;
};