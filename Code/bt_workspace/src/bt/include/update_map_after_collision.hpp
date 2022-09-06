#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "bt_msgs/srv/get_last_map.hpp"
#include "bt_msgs/srv/get_pose.hpp"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "bt_msgs/srv/send_updated_map.hpp"

#define ROBOT_RADIUS 0.45

class UpdateMapAfterCollision : public BT::SyncActionNode
{   
    public:

    UpdateMapAfterCollision(const std::string& name) : BT::SyncActionNode(name, {})
    {
        node_= rclcpp::Node::make_shared("update_map_after_collision");
        map_service_client_ = node_->create_client<bt_msgs::srv::GetLastMap>("get_last_map_service");
        pose_service_client_ = node_->create_client<bt_msgs::srv::GetPose>("get_collision_pose_service");
        // map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_updated", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        // node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map", )
        marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("marker", 1);
        send_map_client_ = node_->create_client<bt_msgs::srv::SendUpdatedMap>("send_updated_map_service");
        clear_costmap_service_client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");

        debug = true;
    }

    bool get_map_service_call(nav_msgs::msg::OccupancyGrid& map)
    {
        while (!map_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetLastCostmap Service not available, waiting again...");
        }

        auto request = std::make_shared<bt_msgs::srv::GetLastMap::Request>();

        auto result = map_service_client_->async_send_request(request);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sending Service Request.");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            map = result.get()->map;
            if(debug)
                RCLCPP_INFO(node_->get_logger(), "Received map with dimensions: width: %d and height: %d", map.info.width, map.info.height);
            return true;
        }
        else
        {
            return false;
        }

        return false;
    }

    bool send_map_service_call(nav_msgs::msg::OccupancyGrid& map)
    {
        while (!send_map_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GetLastCostmap Service not available, waiting again...");
        }

        auto request = std::make_shared<bt_msgs::srv::SendUpdatedMap::Request>();
        request->updated_map = map;

        auto result = send_map_client_->async_send_request(request);

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

    bool get_collision_pose_service_call(geometry_msgs::msg::Pose& collision_pose)
    {
        while (!pose_service_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Collision Pose Service not available, waiting again...");
        }

        auto request = std::make_shared<bt_msgs::srv::GetPose::Request>();

        auto result = pose_service_client_->async_send_request(request);

        if (debug)
            RCLCPP_INFO(node_->get_logger(), "Sending Service Request.");

        if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            collision_pose = result.get()->last_robot_pose;
            return true;
        }
        else
        {
            return false;
        }

        return false;
    }
    
    void convert_data_to_vector(const nav_msgs::msg::OccupancyGrid& map, std::vector<std::vector<int>>& vec_2d)
    {
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Converting to vector 2d.");
        for (size_t i = 0; i < map.info.width-1; i++)
        {   
            // if(debug)
                // RCLCPP_INFO(node_->get_logger(), "i: %d", i);
            for (size_t j = 0; j < map.info.height -1; j++)
            {   
                // if(debug)
                    // RCLCPP_INFO(node_->get_logger(), "j: %d", j);
                vec_2d.at(i).at(j) = map.data.at(i * map.info.height + j);
            }            
        }       
    }

    void publish_marker(geometry_msgs::msg::Point point)
    {
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Publishing marker.");
        visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.id = 1;
        marker.header.stamp = node_->get_clock()->now();
        marker.header.frame_id = "map";
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        
        marker_publisher_->publish(marker);
    }
  
    /**
     * @brief 
     * Determine the position of the collision by creating a vector in the direction of the yaw angle with the length of the robot radius.
     * y axis goes left, x axis goes in front, yaw direction is counterclockwise positive
     * start position is pose.position.x and pose.position.y
     * end position is x + robot_radius * cos(yaw angle) and y + robot_radius * sin(yaw_angle)
     * @param pose 
     * @return geometry_msgs::msg::Point 
     */
    geometry_msgs::msg::Point determine_collision_point(geometry_msgs::msg::Pose pose)
    {   
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Determining Collision Point.");
        tf2::Quaternion quat(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
    
        tf2::Matrix3x3 m(quat);
        double roll,pitch,yaw;
        m.getRPY(roll, pitch, yaw);

        geometry_msgs::msg::Point goal_point;
        goal_point.x = (cos(yaw)*ROBOT_RADIUS) + pose.position.x;
        goal_point.y = (sin(yaw)*ROBOT_RADIUS) + pose.position.y;
        goal_point.z = 0.0;

        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Calculated Collision Point: %f | %f ", goal_point.x, goal_point.y); 

        return goal_point;
    }

    void add_occupation_to_costmap(std::vector<std::vector<int>>& costmap_2d, int x_px, int y_px)
    {
        // modify costmap routine
        // then it should modify the costmap by adding a 5x5 square at the location of the crash  
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Adding occupation to costmap vector."); 
        for (size_t i = x_px -2; i < x_px +2; i++)
        {
            for (size_t j = y_px -2; j < y_px +2; j++)
            {
                costmap_2d.at(i).at(j) = 100; 
            }
            
        }
        
        // costmap_2d.at(x_px).at(y_px) = 100;
        // costmap_2d.at(x_px -1).at(y_px) = 100;
       
    }   

    void convert_vector_to_data(std::vector<std::vector<int>>& costmap_vector_2d, nav_msgs::msg::OccupancyGrid& map)
    {
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "Converting to one dimensional data array and putting it into the map msg.");
        std::vector<int8_t> data((map.info.height * map.info.width), -1);

        for (size_t i = 0; i < costmap_vector_2d.size(); i++) //Rows
        {   
            // if(debug)
                // RCLCPP_INFO(node_->get_logger(), "i: %d", i);
            for (size_t j = 0; j < costmap_vector_2d.at(i).size(); j++) // Columns
            {
                // data.push_back(costmap_vector_2d.at(i).at(j));
                // if(debug)
                    // RCLCPP_INFO(node_->get_logger(), "j: %d", j);
                // if(debug)
                // {
                    // RCLCPP_INFO(node_->get_logger(), "data.size(): %d", data.size());
                    // RCLCPP_INFO(node_->get_logger(), "costmap_vector_2d.size(): %d", costmap_vector_2d.size());
                    // RCLCPP_INFO(node_->get_logger(), "costmap_vector_2d.at(i).size(): %d", costmap_vector_2d.at(i).size());
                    // RCLCPP_INFO(node_->get_logger(), "costmap.at(i).at(j): %d", costmap_vector_2d.at(i).at(j));
                    // RCLCPP_INFO(node_->get_logger(), "costmap_vector_2d.at(i).size()*i + j: %d", costmap_vector_2d.at(i).size()*i + j);
                    // RCLCPP_INFO(node_->get_logger(), "costmap_vector_2d.size(): %d", costmap_vector_2d.size());
                // }
                // if(debug)
                //     RCLCPP_INFO(node_->get_logger(), "Before creash");
                // data.at(map.info.height * i + j) = 100;

                // if(debug)
                // {
                //     RCLCPP_INFO(node_->get_logger(), "data.at should be 100: %d", costmap_vector_2d.at(j).size()*i + j);
                // }
                // data.at((costmap_vector_2d.at(i).size() * i) + j) = costmap_vector_2d.at(i).at(j); // pixel = at.(row).at(column) = 
                // if(debug)
                //     RCLCPP_INFO(node_->get_logger(), "i: %d, j: %d", i, j);

                data.at(map.info.height * i + j ) = costmap_vector_2d.at(i).at(j);                
            }            
        }
        if(data.size() == map.info.width * map.info.height)
        {
            map.data = data;
        }
        else
        {
            if(debug)
            {
                RCLCPP_INFO(node_->get_logger(), "Map Data not compatible. Error while converting.");
                RCLCPP_INFO(node_->get_logger(), "Expected Array of size %d. Got array of size %d", map.info.height * map.info.width, data.size());
            }

        }

    }

    BT::NodeStatus tick() override
    {   
        geometry_msgs::msg::Pose collision_pose;
        nav_msgs::msg::OccupancyGrid map;

        if(!get_map_service_call(map))
            return BT::NodeStatus::FAILURE;

        // std::vector<std::vector<int>> costmap_vector_2d(map.info.width, std::vector<int>(map.info.height, 0));

        std::vector<int> row(map.info.height, 0);
        std::vector<std::vector<int>> costmap_vector_2d(map.info.width, row);

        convert_data_to_vector(map, costmap_vector_2d);

        if(!get_collision_pose_service_call(collision_pose))
            return BT::NodeStatus::FAILURE;

        geometry_msgs::msg::Point goal_point = determine_collision_point(collision_pose);
        publish_marker(goal_point);

        // the origin of the costmap is normally -10, -10
        // the usual resolution is 0.05 m/px which results is 20px/m
        // therefore the the offset is normally 200px in both x and y direction
        // the size of the costmap is normally 384 x 384
        // the formula for converting the goal point into the position in the costmap vector is: 
        // x_px = (x - x_origin) / resolution and y_px = (y-y_origin) / resolution
            
        int x_px = (goal_point.x - map.info.origin.position.x) / map.info.resolution;
        int y_px = (goal_point.y - map.info.origin.position.y) / map.info.resolution;
        
        if(debug)
            RCLCPP_INFO(node_->get_logger(), "X Pixel: %d \n Y Pixel: %d", x_px, y_px);
         
        add_occupation_to_costmap(costmap_vector_2d, x_px, y_px);

        convert_vector_to_data(costmap_vector_2d, map);

        //publish updated map to /map 
        // map_publisher_->publish(map);

        // clear_costmap_service_client_->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());

        if(!send_map_service_call(map))
            return BT::NodeStatus::FAILURE;

        return BT::NodeStatus::SUCCESS;
    }

    private:
    bool debug;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<bt_msgs::srv::GetLastMap>::SharedPtr map_service_client_;
    rclcpp::Client<bt_msgs::srv::GetPose>::SharedPtr pose_service_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_service_client_;
    rclcpp::Client<bt_msgs::srv::SendUpdatedMap>::SharedPtr send_map_client_;
    // rclcpp::Client<
    // std::vector<std::vector<int>> costmap_vector_2d_;

};