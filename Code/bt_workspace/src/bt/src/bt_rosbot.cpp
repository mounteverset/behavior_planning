#include "bt_rosbot.hpp"
#include <filesystem>

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
    // BT::PortsList lidar_exec_check_ports = {BT::InputPort<bool>("is_lidar_running")};
    factory.registerNodeType<LidarExecutionCheck>("LidarExecutionCheck");
    factory.registerNodeType<AccelToNormalSpeed>("AccelToNormalSpeed");
    factory.registerNodeType<DecelToMinDrivingSpeed>("DecelToMinDrivingSpeed");
    factory.registerNodeType<CompleteStop>("CompleteStop");
    factory.registerNodeType<RestartLidar>("RestartLidar");

    factory.registerNodeType<ImuExecutionCheck>("ImuExecutionCheck");
    factory.registerNodeType<RestartImu>("RestartImu");

    factory.registerNodeType<OdomExecutionCheck>("OdomExecutionCheck");
    factory.registerNodeType<RestartOdom>("RestartOdom");

    factory.registerNodeType<CollisionCheck>("CollisionCheck");
    factory.registerNodeType<OrientationCheck>("OrientationCheck");
    factory.registerNodeType<ReverseCmdVel>("ReverseCmdVel");

    factory.registerNodeType<EnableCmdVelOverride>("EnableCmdVelOverride");
    factory.registerNodeType<DisableCmdVelOverride>("DisableCmdVelOverride");
    factory.registerNodeType<SlamExecutionCheck>("SlamExecutionCheck");
    factory.registerNodeType<ResetOccupancyMap>("ResetOccupancyMap");
    factory.registerNodeType<UpdateMapAfterCollision>("UpdateMapAfterCollision");
    factory.registerNodeType<SaveUpdatedMap>("SaveUpdatedMap");
    factory.registerNodeType<LoadUpdatedMap>("LoadUpdatedMap");

    factory.registerNodeType<BatterySufficientCheck>("BatterySufficientCheck");

    factory.registerNodeType<CancelNavGoal>("CancelNavGoal");
    factory.registerNodeType<RepublishLastGoal>("RepublishLastGoal");

    factory.registerNodeType<GlobalPlannerExecutionCheck>("GlobalPlannerExecutionCheck");
    factory.registerNodeType<RestartGlobalPlanner>("RestartGlobalPlanner");
    factory.registerNodeType<PathPossibleCheck>("PathPossibleCheck");
    factory.registerNodeType<PublishCloserGoal>("PublishCloserGoal");

    RCLCPP_INFO(rclcpp::get_logger("root"), "Creating BT"); 
    // std::string a = std::filesystem::current_path();
    // RCLCPP_INFO(rclcpp::get_logger("root"), a.c_str()); 
    auto tree = factory.createTreeFromFile("./src/bt/resources/rosbot_bt_lite.xml");
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

        switch (status)
        {
            case BT::NodeStatus::FAILURE:
            {
                RCLCPP_ERROR(rclcpp::get_logger("root"), "BT Status: FAILURE");
                break;
            }
            case BT::NodeStatus::SUCCESS:
            {
                RCLCPP_INFO(rclcpp::get_logger("root"), "BT Status: SUCCESS");
                break;
            }
            case BT::NodeStatus::RUNNING:
            {
                RCLCPP_INFO(rclcpp::get_logger("root"), "BT Status: RUNNING");
                break;
            }
            case BT::NodeStatus::IDLE:
            {
                RCLCPP_INFO(rclcpp::get_logger("root"), "BT Status: IDLE");
                break;
            }
        }

        // RCLCPP_INFO(rclcpp::get_logger("root"), "BT Status");  
        // RCLCPP_INFO(rclcpp::get_logger("root"), "%d", ); 
        // std::cout << "Status: " << status << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // tree.sleep(std::chrono::milliseconds(50));
    }

    

    return 0;
}
