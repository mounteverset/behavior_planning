#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "behaviortree_cpp_v3/action_node.h"


#define MAX_ALLOWED_TIME_DIFFERENCE 1.0

class ExecutionChecker : public BT::StatefulActionNode
{
    public:
        ExecutionChecker(const std::string &name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config)
        {
            // Initialize ROS related Variables
            node_= rclcpp::Node::make_shared("execution_checker");
            rclcpp::QoS topics_qos = rclcpp::SystemDefaultsQoS();      
            topics_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
            subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>("scan", topics_qos, std::bind(&ExecutionChecker::lidar_callback, this, std::placeholders::_1));
            executor_.add_node(node_);
            last_msg_received_ = node_->get_clock()->now();


            //Init BT related variables
            halt_requested_ = false;
            is_lidar_running_ = false;

            halt_requested_ = false;
            // 
            time_difference = 0.0;

            // Debug Flags
            debug = false;
            debug_callback = true;
        }

        // static BT::PortsList providedPorts()
        // {
        //     return {BT::OutputPort<bool>("lidar_running")};
        // }

        BT::NodeStatus onStart() override
        {
            while (true)
            {
                //executor_.spin_some(std::chrono::milliseconds(250)); // std::chrono::nanoseconds(25000000000) checks every 500ms for halt requested

                executor_.spin_once();
                
                time_difference = (node_->get_clock()->now().seconds() - last_msg_received_.seconds());

                if (time_difference > MAX_ALLOWED_TIME_DIFFERENCE)
                {
                    is_lidar_running_ = false;
                }

                if(debug)
                {   
                    RCLCPP_INFO(node_->get_logger(), "Time Difference in sec: %f",  time_difference);
                    RCLCPP_INFO(node_->get_logger(), "Output Port written to: ",  is_lidar_running_);
                    RCLCPP_INFO(node_->get_logger(), (is_lidar_running_) ? "true" : "false");
                }
                setOutput("is_lidar_running", is_lidar_running_);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(300));

                if (first_run)
                    return BT::NodeStatus::RUNNING;
            }  
        }

        BT::NodeStatus onRunning() override
        {   
            first_run =true;
            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override
        {   
            executor_.spin_once();

            if(debug)
                RCLCPP_INFO(node_->get_logger(), "Halt requested. Node will shutdown.");

            first_run = true;
        }

        // BT::NodeStatus tick()
        // {   

        //     while(!halt_requested_)
        //     {
        //         executor_.spin_some(std::chrono::milliseconds(250)); // std::chrono::nanoseconds(25000000000) checks every 500ms for halt requested
                
        //         time_difference = (node_->get_clock()->now().seconds() - last_msg_received_.seconds());

        //         if (time_difference > MAX_ALLOWED_TIME_DIFFERENCE)
        //         {
        //             is_lidar_running_ = false;
        //         }

        //         if(debug)
        //         {   
        //             RCLCPP_INFO(node_->get_logger(), "Time Difference in sec: %f",  time_difference);
        //             RCLCPP_INFO(node_->get_logger(), "Output Port written to: ",  is_lidar_running_);
        //             RCLCPP_INFO(node_->get_logger(), (is_lidar_running_) ? "true" : "false");
        //         }
        //         setOutput("is_lidar_running", is_lidar_running_);
        //     }
        // }
        //     return BT::NodeStatus::FAILURE;
        // }

        // void halt() override
        // {
        //     halt_requested_ = true;
        // }

        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {   
            if(debug_callback)
            {
                RCLCPP_INFO(node_->get_logger(), "Received a Scan Message");
                RCLCPP_INFO(node_->get_logger(), "last_msg_received at: %f", last_msg_received_.seconds());
                RCLCPP_INFO(node_->get_logger(), "msg.header.stamp at: %f", msg->header.stamp.sec);
            }

            // if((node_->get_clock()->now().seconds() - last_msg_received_.seconds()) < 1.5)
            // {
            //     if(debug_callback)
            //         RCLCPP_INFO(node_->get_logger(), "Lidar active");
            //     is_lidar_running_ = true;
            // }
            is_lidar_running_ = true;
            last_msg_received_ = node_->get_clock()->now();
                
            // }
            // else 
            // {
            //     is_lidar_running_ = false;
            // }

            if(debug_callback)
            {
                RCLCPP_INFO(node_->get_logger(), "Output Port written to: ",  is_lidar_running_);
                RCLCPP_INFO(node_->get_logger(), (is_lidar_running_) ? "true" : "false");
            }

            setOutput("is_lidar_running", is_lidar_running_);

            if(debug)
                RCLCPP_INFO(node_->get_logger(), "last_msg_time when callback finished: %f", last_msg_received_.seconds());
        }

    
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Time last_msg_received_;

        bool is_lidar_running_; 

        bool halt_requested_;

        bool first_run;

        float time_difference;
        // float allowed_time_difference;

        bool debug;
        bool debug_callback;
};

