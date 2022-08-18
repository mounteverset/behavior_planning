#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "bt_msgs/srv/pub_cmd_vel.hpp"
#include "bt_msgs/srv/get_twist_array.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

#define MIN_DRIVING_SPEED 0.05

/**
 * @brief This Node blocks the execution of the rest of the tree and publishes the last navigation commands in reverse with reduced speed
 * 
 */
class ReverseCmdVel : public BT::SyncActionNode
{   
    public:

    ReverseCmdVel(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("reverse_cmd_vel");
        
        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Constructor ReverseCmdVel");



        service_client_pub_cmd_vel = node_->create_client<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service");
        request_pub_cmd_vel = std::make_shared<bt_msgs::srv::PubCmdVel_Request>();


        if (debug)
            RCLCPP_INFO(node_->get_logger(), "1");
        service_client_get_twist_array = node_->create_client<bt_msgs::srv::GetTwistArray>("get_last_cmd_vels");

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "2");

        service_client_set_override_flag = node_->create_client<rcl_interfaces::srv::SetParameters>("cmd_vel_decision_gate/set_parameters");

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "3");

        param.name = "bt_override";       
        param.value.bool_value = true;
        parameters.push_back(param);

        // request_set_parameter->parameters.push_back(param);
        // request_set_parameter->parameters.emplace(request_set_parameter->parameters.rbegin(), param);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "4");

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Finished Constructor ReverseCmdVel");
    }

    bool pub_cmd_vel_service_call()
    {
        auto result = service_client_pub_cmd_vel->async_send_request(request_pub_cmd_vel);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sent Service Request");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Get the twist array service call object, the oldest msg is at the start 
     * 
     * @return true 
     * @return false 
     */
    bool get_twist_array_service_call()
    {
        auto result = service_client_get_twist_array->async_send_request(std::make_shared<bt_msgs::srv::GetTwistArray_Request>());

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            twist_array = result.get()->cmd_vel_array;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool set_override_flag_service_call()
    {
        // auto request = service_client_set_override_flag->
        request_set_parameter->parameters = parameters;

        auto result = service_client_set_override_flag->async_send_request(request_set_parameter);

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief Calls the pub_cmd_vel service for every of the saved cmd_vel of the last 2 seconds
     * Modifies the cmd_vel to use a reduced speed when reversing
     * This leads to a longer publishing time than 2 seconds. 
     * Default publish frequency of controller server is 20Hz, leading to 1 cmd_vel every 50ms
     * Reducing the speed and angular vel, leads to decreasing the Hz by the same factor
     *
     * This function blocks the rest of the behaviour tree, because it does not exit after the service call
     * 
     * 
     * @return true 
     * @return false 
     */
    bool reverse_cmd_vel()
    {   
        for (size_t i = twist_array.size()-1 ; i > 0; i--)
        {   
            // TODO:
            // Make a case when robot is turning in place without linear motion.
            // Make a case when cmd_vel is lower than MIN_DRIVING_SPEED
            float factor = twist_array.back().linear.x / MIN_DRIVING_SPEED;
            
            //request_pub_cmd_vel->cmd_vel.linear.x = pow(twist_array.back().linear.x, -1) * MIN_DRIVING_SPEED * -1; // v_alt^(-1) * -MIN_SPEED = Minspeed invertiert
            request_pub_cmd_vel->cmd_vel.linear.x = (-MIN_DRIVING_SPEED / abs(twist_array[i].linear.x)) * twist_array.at(i).linear.x; // This always drives with min_speed in the opposite direction
            request_pub_cmd_vel->cmd_vel.angular.z = twist_array.at(i).angular.z / factor;
            float pub_duration = 0.05 * abs(factor); // 50ms = 20Hz = rate of controller server. abs bc time has to be positive
            request_pub_cmd_vel->time_in_seconds = pub_duration;
            
            if(!pub_cmd_vel_service_call())
                return false;
            
            rclcpp::sleep_for(std::chrono::milliseconds(int(pub_duration*1000))); // convert milliseconds to seconds and cast to in. might lose some milliseconds when casting

        }
        return true;
    }


    BT::NodeStatus tick() override
    {
        if(!get_twist_array_service_call())
            return BT::NodeStatus::FAILURE;

        if(!set_override_flag_service_call())
            return BT::NodeStatus::FAILURE;

        if(!reverse_cmd_vel())
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    private:
    bool debug = true;
    std::shared_ptr<rclcpp::Node> node_;

    rcl_interfaces::msg::Parameter param;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr service_client_set_override_flag;
    std::vector<rcl_interfaces::msg::Parameter> parameters;
    std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request_set_parameter;

    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Client<bt_msgs::srv::PubCmdVel>::SharedPtr service_client_pub_cmd_vel;
    std::shared_ptr<bt_msgs::srv::PubCmdVel::Request> request_pub_cmd_vel;

    rclcpp::Client<bt_msgs::srv::GetTwistArray>::SharedPtr service_client_get_twist_array;
    std::shared_ptr<bt_msgs::srv::GetTwistArray_Response> response_get_twist_array;
    std::vector<geometry_msgs::msg::Twist> twist_array;
};