#ifndef TERRAIN_COSTMAP_PLUGIN__TRAVERSABILITY_LAYER_HPP_
#define TERRAIN_COSTMAP_PLUGIN__TRAVERSABILITY_LAYER_HPP_

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

namespace terrain_costmap_plugin
{

unsigned char traversabilityToCost(
  float t, double free_threshold, double lethal_threshold, unsigned char max_non_lethal);

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void activate() override;
  void deactivate() override;
  bool isClearable() override {return false;}

private:
  void mapCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr msg);
  void paintLatestMap(double * min_x, double * min_y, double * max_x, double * max_y);

  std::string topic_;
  std::string layer_name_;
  double free_threshold_{0.3};
  double lethal_threshold_{0.95};
  int max_non_lethal_cost_{252};
  double transform_tolerance_{0.3};
  bool use_maximum_{true};

  std::mutex mutex_;
  grid_map_msgs::msg::GridMap::ConstSharedPtr latest_;
  bool new_map_{false};

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
};

}

#endif
