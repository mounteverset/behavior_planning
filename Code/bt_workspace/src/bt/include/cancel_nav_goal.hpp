#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class CancelNavGoal : public BT::SyncActionNode
{   
    public:

        CancelNavGoal(const std::string& name) : BT::SyncActionNode(name, {})
        {
            node_= rclcpp::Node::make_shared("complete_stop");

            action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
        }

        // void cancel_callback()
        // {

        // }

        BT::NodeStatus tick() override
        {
            auto response = action_client_->async_cancel_all_goals();
            //auto return_code = response.get()->return_code;
            return BT::NodeStatus::SUCCESS;
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
        // auto goal_handle_;

};


