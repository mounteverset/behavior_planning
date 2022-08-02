#include "bt_lidar.hpp"
#include "complete_stop.hpp"
#include "decel_to_min_driving_speed.hpp"
#include "lidar_execution_check.hpp"
#include "restart_lidar.hpp"


int main(int argc, const char* argv[])
{   
    //node_ptr = std::make_shared<BtHelperNode>();
    rclcpp::init(argc, argv);
    
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<LidarExecutionCheck>("LidarExecutionCheck");
    factory.registerNodeType<DecelToMinDrivingSpeed>("DecelToMinDrivingSpeed");
    factory.registerNodeType<CompleteStop>("CompleteStop");
    factory.registerNodeType<RestartLidar>("RestartLidar");
    

    auto tree = factory.createTreeFromFile("/home/luke/behavioural-planning/Code/bt_workspace/src/bt/resources/lidar_bt.xml");
    // BT::NodeStatus status;
    // while(true)
    // {   
        std::cout << "Ticking root" << std::endl;
        // status = 
        tree.tickRoot();

        // std::cout << "Status: " << status << std::endl;
    // }

    return 0;
}
