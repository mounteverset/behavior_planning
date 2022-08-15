#include "bt_lidar.hpp"
#include "complete_stop.hpp"
#include "decel_to_min_driving_speed.hpp"
#include "lidar_execution_check.hpp"
#include "restart_lidar.hpp"
#include "execution_checker.hpp"


#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/behavior_tree.h"



int main(int argc, const char* argv[])
{   
    
    //node_ptr = std::make_shared<BtHelperNode>();
    rclcpp::init(argc, argv);
    // auto nh = rclcpp::Node::make_shared("bt_lidar_master");
    // rclcpp::Parameter sim_time_param("use_sim_time", rclcpp::ParameterValue(true));
    // // rclcpp::Node::set_parameter(sim_time_param)
    // nh->set_parameter(sim_time_param);
    
    BT::BehaviorTreeFactory factory;

    // BT::PortsList exec_checker_ports = {BT::OutputPort<bool>("is_lidar_running")};
    // factory.registerNodeType<ExecutionChecker>("ExecutionChecker", exec_checker_ports);
    RCLCPP_INFO(rclcpp::get_logger("root"), "Creating Nodes"); 
    BT::PortsList lidar_exec_check_ports = {BT::InputPort<bool>("is_lidar_running")};
    factory.registerNodeType<LidarExecutionCheck>("LidarExecutionCheck");
    factory.registerNodeType<DecelToMinDrivingSpeed>("DecelToMinDrivingSpeed");
    factory.registerNodeType<CompleteStop>("CompleteStop");
    factory.registerNodeType<RestartLidar>("RestartLidar");
    RCLCPP_INFO(rclcpp::get_logger("root"), "Creating BT"); 
    auto tree = factory.createTreeFromFile("/home/luke/behavioural-planning/Code/bt_workspace/src/bt/resources/lidar_bt.xml");
    RCLCPP_INFO(rclcpp::get_logger("root"), "Created BT");
    RCLCPP_INFO(rclcpp::get_logger("root"), "Creating BT Publisher"); 

    BT::PublisherZMQ publisher_zqm(tree, 25, 1668, 1669);
    BT::NodeStatus status;

    BT::FileLogger logger_file(tree, "bt_lidar_trace.fbl");
    RCLCPP_INFO(rclcpp::get_logger("root"), "BT Status"); 
    while(true)
    { 
        
        RCLCPP_INFO(rclcpp::get_logger("root"), "Ticking root");  

        status = tree.tickRoot();

        RCLCPP_INFO(rclcpp::get_logger("root"), "BT Status");  
        RCLCPP_INFO(rclcpp::get_logger("root"), "%d", status); 
        // std::cout << "Status: " << status << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // tree.sleep(std::chrono::milliseconds(50));
    }

    

    return 0;
}
