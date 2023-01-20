#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/srv/save_map.hpp"

class SaveUpdatedMap : public BT::SyncActionNode
{   
    public:

    SaveUpdatedMap(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("save_updated_map");
        map_service_client_ = node_->create_client<nav2_msgs::srv::SaveMap>("map_saver/save_map");

        //clear_costmap_service_client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");

        debug = true;
    }

    bool save_map_service_call()
    {
        while (!map_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("save_updated_map"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("save_updated_map"), "SaveMap Service not available, waiting again...");
        }

        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_topic = "map_updated";
        request->map_url = "./src/bt/maps/map_updated";

        auto result = map_service_client_->async_send_request(request);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sending Service Request.");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            return true;
        }
        else
        {
            return false;
        }

        return false;
    }

    BT::NodeStatus tick() override
    {   
        if(!save_map_service_call())
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    private:
    bool debug;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_service_client_;
    // rclcpp::Client<bt_msgs::srv::GetPose>::SharedPtr pose_service_client_;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    // rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_service_client_;
    // rclcpp::Client<
    // std::vector<std::vector<int>> costmap_vector_2d_;

};