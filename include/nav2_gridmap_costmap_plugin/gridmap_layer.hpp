#ifndef GRIDMAP_LAYER_HPP_
#define GRIDMAP_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "gridmap_handler.hpp"

namespace nav2_gridmap_costmap_plugin
{

class GridmapLayer : public nav2_costmap_2d::Layer
{
public:
  GridmapLayer();
  ~GridmapLayer();

  virtual void onInitialize();

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, 
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, 
    int min_j, 
    int max_i, 
    int max_j);

  virtual void reset();
  virtual void onFootprintChanged();

private:

  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  GridMapHandler grid_map_handler_;
  std::shared_ptr<grid_map::GridMap> grid_map_;

  double slope_traversability_delta_;
  double slope_traversability_alpha_;
  double slope_traversability_lambda_;


};

}

#endif
