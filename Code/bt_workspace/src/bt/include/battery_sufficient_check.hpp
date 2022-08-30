#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"

#include "bt_msgs/srv/get_charge.hpp"
#include "bt_msgs/srv/get_distance.hpp"

/**
 * @brief Returns Success if the battery has enough charge to reach to goal
 * 
 */
class BatterySufficientCheck : public BT::ConditionNode
{
public:

    BatterySufficientCheck(const std::string& name) : BT::ConditionNode(name, {})
    {   
        debug = true;

        node_= rclcpp::Node::make_shared("battery_sufficient_check");

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor BatterySufficientCheck");

        distance_service_client_ = node_->create_client<bt_msgs::srv::GetDistance>("goal_distance_service");
        charge_service_client_ = node_->create_client<bt_msgs::srv::GetCharge>("get_charge_service");
        //request_ = std::make_shared<std_srvs::srv::SetBool::Request>();
        //request_->data = true;
        distance_request_ = std::make_shared<bt_msgs::srv::GetDistance::Request>();
        charge_request_ = std::make_shared<bt_msgs::srv::GetCharge::Request>();

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor BatterySufficientCheck finished.");

        avrg_speed_ = 0.2;
        safety_factor_ = 1.5;
    }

    bool distance_service_call(float &distance)
    {
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Distance Service Call");
        while (!distance_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance Service not available, waiting again...");
        }

        auto distance_result = distance_service_client_->async_send_request(distance_request_);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sent Service Request");

        if(rclcpp::spin_until_future_complete(node_, distance_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            distance = distance_result.get()->distance_in_meter;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool charge_service_call(float &charge, float &idle_decrease, float &drive_decrease)
    {   
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Charge Service Call");

        while (!charge_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Charge Service not available, waiting again...");
        }

        auto charge_result = charge_service_client_->async_send_request(charge_request_);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sent Service Request");

        if(rclcpp::spin_until_future_complete(node_, charge_result) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            charge = charge_result.get()->charge;
            idle_decrease = charge_result.get()->idle_decrease_per_sec;
            drive_decrease = charge_result.get()->drive_decrease_per_sec;
            return true;
        }
        else
        {
            return false;
        }
    }

    BT::NodeStatus tick() override
    {
        float distance;

        if(!distance_service_call(distance))
            return BT::NodeStatus::FAILURE;

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Distance to Goal: %f", distance);

        if(distance == 0.0)
            return BT::NodeStatus::SUCCESS;
        
        float charge, idle_decrease, drive_decrease;

        if(!charge_service_call(charge, idle_decrease, drive_decrease))
            return BT::NodeStatus::FAILURE;
        
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Charge: %f", charge);

        float time = distance / avrg_speed_;

        float consumption = time * (idle_decrease + drive_decrease);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Consumption: %f", consumption);

        if (charge >= (consumption * safety_factor_))
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

    rclcpp::Client<bt_msgs::srv::GetDistance>::SharedPtr distance_service_client_;
    rclcpp::Client<bt_msgs::srv::GetCharge>::SharedPtr charge_service_client_;

    std::shared_ptr<bt_msgs::srv::GetDistance::Request> distance_request_;
    std::shared_ptr<bt_msgs::srv::GetCharge::Request> charge_request_;

    float avrg_speed_;
    float safety_factor_;
    
    bool debug;
};