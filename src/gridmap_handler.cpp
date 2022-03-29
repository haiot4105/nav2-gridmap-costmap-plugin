#include "nav2_gridmap_costmap_plugin/gridmap_handler.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_gridmap_costmap_plugin
{

void GridMapHandler::on_initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent,
  const std::string & node_name,
  const std::shared_ptr<grid_map::GridMap> & grid_map)
{
  parent_ = parent;
  node_name_ = node_name;
  grid_map_ = grid_map;

  get_params();
  create_subscribers();
  RCLCPP_INFO(logger_, "Configured");
}

void GridMapHandler::get_params()
{
    std::string param_name = "grid_map_topic";
    std::string name = node_name_ + '.' + param_name;
  	parent_->declare_parameter(name, rclcpp::ParameterValue(std::string("elevation_map")));
	  parent_->get_parameter(name, input_topic_);
}

void GridMapHandler::create_subscribers()
{
    subscriber_ = parent_->create_subscription<grid_map_msgs::msg::GridMap>(input_topic_, 1, std::bind(&GridMapHandler::grid_map_callback, this, std::placeholders::_1));
}

void GridMapHandler::grid_map_callback(const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  std::lock_guard<std::recursive_mutex> lk(access_);
  grid_map::GridMapRosConverter::fromMessage(*message, *grid_map_);
}


} 
