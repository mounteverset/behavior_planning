#include "behaviortree_cpp_v3/bt_factory.h"

class RobotWatchdog : public BT::SyncActionNode
{
    public:
        RobotWatchdog(const std::string& name) : BT::SyncActionNode(name, {})
        {

        }

        BT::NodeStatus tick() override
        {
            std::cout << "Robot Watchdog: " << this->name() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};

class LidarExecutionCheck: public BT::ConditionNode
{
    public: 
    LidarExecutionCheck(const std::string& name): BT::ConditionNode(name, {})
    {

    }

    BT::NodeStatus tick() override
    {
        std::cout << "Lidar is alive says: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main()
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<LidarExecutionCheck>("LidarExecutionCheck");

    auto tree = factory.createTreeFromFile("/home/levers/behavioural-planning/Code/bt_workspace/src/bt/resources/bt.xml");

    tree.tickRoot();

    return 0;
}