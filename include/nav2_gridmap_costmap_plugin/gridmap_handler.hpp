#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <grid_map_ros/grid_map_ros.hpp>

#include <mutex>

namespace nav2_gridmap_costmap_plugin
{

class GridMapHandler
{

public:
    GridMapHandler() = default;
    ~GridMapHandler() = default;

    void on_initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent,
    const std::string & node_name,
    const std::shared_ptr<grid_map::GridMap> & grid_map);

    void reset() {}

    auto get_mutex() -> std::recursive_mutex & 
    {
        return access_;
    }

private:
  void get_params();
  void create_subscribers();
  void grid_map_callback(const grid_map_msgs::msg::GridMap::SharedPtr message);

private:
  std::recursive_mutex access_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  std::string input_topic_;
  std::shared_ptr<grid_map::GridMap> grid_map_;

  rclcpp::Logger logger_{rclcpp::get_logger("gridmap_layer grid_map_handler")};
  std::shared_ptr<rclcpp::Subscription<grid_map_msgs::msg::GridMap>> subscriber_;
};

}
