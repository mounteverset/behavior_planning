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

        cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_bt", 10);

        // service_client_pub_cmd_vel = node_->create_client<bt_msgs::srv::PubCmdVel>("pub_cmd_vel_service");
        // request_pub_cmd_vel = std::make_shared<bt_msgs::srv::PubCmdVel_Request>();

        service_client_get_twist_array = node_->create_client<bt_msgs::srv::GetTwistArray>("get_last_cmd_velocities");

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Finished Constructor ReverseCmdVel");
    }

    // bool pub_cmd_vel_service_call()
    // {   
    //     while (!service_client_get_twist_array->wait_for_service(std::chrono::seconds(1)))
    //     {
    //         if (!rclcpp::ok()) 
    //         {
    //             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //             return false;
    //         }
    //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Twist Array Service not available, waiting again...");
    //     }

    //     auto result = service_client_pub_cmd_vel->async_send_request(request_pub_cmd_vel);

    //     if (debug)
    //         RCLCPP_INFO(node_->get_logger(), "Sent Service Request");

    //     if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    //     {
    //         return true;
    //     }
    //     else
    //     {
    //         return false;
    //     }
    // }

    /**
     * @brief Get the twist array service call object, the oldest msg is at the start 
     * 
     * @return true 
     * @return false 
     */
    bool get_twist_array_service_call()
    {   
        while (!service_client_get_twist_array->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Twist Array Service not available, waiting again...");
        }

        auto request = std::make_shared<bt_msgs::srv::GetTwistArray_Request>();

        auto result = service_client_get_twist_array->async_send_request(request);

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {   
            twist_array = result.get()->cmd_vel_array;
            if (debug)
                RCLCPP_INFO(node_->get_logger(), "Reveived a vector of length: %d", twist_array.size());
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
            geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
            float factor = twist_array.at(i).linear.x / MIN_DRIVING_SPEED;
            float pub_duration;
            
            if (factor != 0)
            {

                //request_pub_cmd_vel->cmd_vel.linear.x = pow(twist_array.back().linear.x, -1) * MIN_DRIVING_SPEED * -1; // v_alt^(-1) * -MIN_SPEED = Minspeed invertiert
                cmd_vel.linear.x = (-MIN_DRIVING_SPEED / abs(twist_array.at(i).linear.x)) * twist_array.at(i).linear.x; // This always drives with min_speed in the opposite direction
                cmd_vel.angular.z = -twist_array.at(i).angular.z / factor;
                pub_duration = 0.05 * abs(factor); // 50ms = 20Hz = rate of controller server. abs bc time has to be positive
                
                if(debug)
                {
                    RCLCPP_INFO(rclcpp::get_logger("reverse_cmd_vel"), "Speed Reduction factor: %f", factor);
                    RCLCPP_INFO(rclcpp::get_logger("reverse_cmd_vel"), "Calculated Cmd_Vel.lin.x: %f", cmd_vel.linear.x);
                    RCLCPP_INFO(rclcpp::get_logger("reverse_cmd_vel"), "Calculated cmd_vel.ang.z: %f", cmd_vel.angular.z);
                    RCLCPP_INFO(rclcpp::get_logger("reverse_cmd_vel"), "Pub Duration: %f", pub_duration);
                }
            }
            else if (factor == 0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -twist_array.at(i).angular.z;
            }
            
            cmd_vel_publisher_->publish(cmd_vel);

            rclcpp::sleep_for(std::chrono::milliseconds(int(pub_duration*1000))); // convert milliseconds to seconds and cast to int. might lose some milliseconds when casting

        }
        return true;
    }


    BT::NodeStatus tick() override
    {
        

        if(!get_twist_array_service_call())
            return BT::NodeStatus::FAILURE;

        if(!reverse_cmd_vel())
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    private:
    bool debug = true;
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // rclcpp::Client<bt_msgs::srv::PubCmdVel>::SharedPtr service_client_pub_cmd_vel;
    // std::shared_ptr<bt_msgs::srv::PubCmdVel::Request> request_pub_cmd_vel;

    rclcpp::Client<bt_msgs::srv::GetTwistArray>::SharedPtr service_client_get_twist_array;
    std::shared_ptr<bt_msgs::srv::GetTwistArray_Response> response_get_twist_array;
    std::vector<geometry_msgs::msg::Twist> twist_array;
};