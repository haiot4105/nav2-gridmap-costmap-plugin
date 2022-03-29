
#include "nav2_gridmap_costmap_plugin/gridmap_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_gridmap_costmap_plugin
{


GridmapLayer::GridmapLayer()
: last_min_x_(std::numeric_limits<float>::lowest()),
  last_min_y_(std::numeric_limits<float>::lowest()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
	// TODO
}



GridmapLayer::~GridmapLayer()
{
	// TODO
}


void GridmapLayer::onInitialize()
{
	// TODO
	declareParameter("enabled", rclcpp::ParameterValue(true));
	node_->get_parameter(name_ + "." + "enabled", enabled_);

	declareParameter("slope_traversability_delta", rclcpp::ParameterValue(0.1));
	declareParameter("slope_traversability_alpha", rclcpp::ParameterValue(0.5));
	declareParameter("slope_traversability_lambda", rclcpp::ParameterValue(3.0));
	
	node_->get_parameter(name_ + "." + "slope_traversability_delta", slope_traversability_delta_);
	node_->get_parameter(name_ + "." + "slope_traversability_alpha", slope_traversability_alpha_);
	node_->get_parameter(name_ + "." + "slope_traversability_lambda", slope_traversability_lambda_);

	
	grid_map_ = std::make_shared<grid_map::GridMap>();
	grid_map_handler_.on_initialize(node_, name_, grid_map_);


	current_ = true;
}


void GridmapLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, 
  double * min_x,
  double * min_y, 
  double * max_x, 
  double * max_y)
{

	*min_x = std::numeric_limits<float>::lowest();
	*min_y = std::numeric_limits<float>::lowest();
	*max_x = std::numeric_limits<float>::max();
	*max_y = std::numeric_limits<float>::max();
}


void GridmapLayer::onFootprintChanged()
{
	// TODO
	RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "GridmapLayer::onFootprintChanged(): footprint updated");
}


void GridmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, 
  int min_i, 
  int min_j,
  int max_i,
  int max_j)
{

	if (!enabled_) 
	{
		return;
	}
	auto gm_size = grid_map_->getSize();

	std::lock_guard<std::recursive_mutex> lk(grid_map_handler_.get_mutex());
	std::lock_guard<std::recursive_mutex> lk2(*master_grid.getMutex());

	unsigned char * master_array = master_grid.getCharMap();
	unsigned int size_x = master_grid.getSizeInCellsX();
	unsigned int size_y = master_grid.getSizeInCellsY();


	min_i = std::max(0, min_i);
	min_j = std::max(0, min_j);
	max_i = std::min(static_cast<int>(size_x), max_i);
	max_j = std::min(static_cast<int>(size_y), max_j);


	auto isUntraversable = [this](const grid_map::Index &cell, bool &unknown)
	{
		std::string layer_name = "elevation";
		grid_map::Position cell_pos, p1, p2;
		grid_map::Index i1, i2;
		auto slope_xy = std::array<double, 2>();
		grid_map_->getPosition(cell, cell_pos);


		for (size_t i = 0; i < 2; i++)
		{
			p1 = cell_pos;
			p2 = cell_pos;
			p1[i] -= this->slope_traversability_delta_;
			p2[i] += this->slope_traversability_delta_;

			if (grid_map_->getIndex(p1, i1) and grid_map_->getIndex(p2, i2))
			{
				slope_xy[i] = std::abs((grid_map_->at(layer_name, i1) - grid_map_->at(layer_name, i2))) / (2 * this->slope_traversability_delta_);
			}
			else
			{
				unknown = true;
				return true;
			}
		}

		double slope = this->slope_traversability_alpha_ * slope_xy[0] +
						(1 - slope_traversability_alpha_) * slope_xy[1];

		double traversable = std::exp(-this->slope_traversability_lambda_ * slope);

		if (std::isnan(traversable))
		{
			unknown = true;
			return true;
		}
		unknown = false;
		return not(traversable > 0.5);
	};


	for (int j = min_j; j < max_j; j++) 
	{
		for (int i = min_i; i < max_i; i++)
		{	

			//TODO transform 

			double x, y;
			unsigned char new_cost = NO_INFORMATION;


			master_grid.mapToWorld(i, j, x, y);

			grid_map::Position point(x, y);
			grid_map:: Index point_index;
			if (grid_map_->getIndex(point, point_index))
			{
				bool isunknown;
				bool isobstacle = isUntraversable(point_index, isunknown);
				if(isunknown)
				{
					new_cost = NO_INFORMATION;
				}
				else if(isobstacle)
				{
					new_cost = LETHAL_OBSTACLE;
				}
				else
				{
					new_cost = FREE_SPACE;
				}
			}
			
			int index = master_grid.getIndex(i, j);
			master_array[index] = new_cost;
		}
	}
}

void GridmapLayer::reset() 
{
	// TODO
	grid_map_handler_.reset();
}


}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gridmap_costmap_plugin::GridmapLayer, nav2_costmap_2d::Layer)
