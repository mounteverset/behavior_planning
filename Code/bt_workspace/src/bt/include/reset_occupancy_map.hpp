#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"

/**
 * @brief 
 * 
 */
class ResetOccupancyMap : public BT::ConditionNode
{
public:

    ResetOccupancyMap(const std::string& name) : BT::ConditionNode(name, {})
    {   
        node_= rclcpp::Node::make_shared("reset_ogm");
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor ResetOccupancyMap");

        // service_client_ = node_->create_client<std_srvs::srv::SetBool>("orientation_check_service");
        // request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        // request_->data = true;

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor ResetOccupancyMap finished.");
        // debug = true;
        debug = false;
    }

    BT::NodeStatus tick() override
    {
    //     while (!service_client_->wait_for_service(std::chrono::seconds(1)))
    //     {
    //         if (!rclcpp::ok()) 
    //         {
    //             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //             return BT::NodeStatus::FAILURE;
    //         }
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Orientation Checker Service not available, waiting again...");
    //     }
        
    //     auto result = service_client_->async_send_request(request_);

    //     if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    //     {
    //         all_wheels_on_the_ground = result.get()->success;
    //     }
    //     else
    //     {
    //         return BT::NodeStatus::FAILURE;
    //     }

    //     // getInput("is_lidar_running", is_lidar_running_);
    //     // (is_lidar_running_) ? is_lidar_running_ = true : is_lidar_running_ = false;

    //     if(debug)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("orientation_check"), "Robot is standing with 4 wheels: ");
    //         RCLCPP_INFO(rclcpp::get_logger("orientation_check"), (all_wheels_on_the_ground) ? "true" : "false");
    //     }

    //     if (all_wheels_on_the_ground)
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else 
    //     {
    //         return BT::NodeStatus::FAILURE;
    //     }
        return BT::NodeStatus::SUCCESS;
    }


private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;
    std::shared_ptr<std_srvs::srv::SetBool::Request> request_;
    bool debug;

};