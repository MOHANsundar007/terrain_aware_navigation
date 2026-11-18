#include "terrain_costmap_plugin/traversability_layer.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace terrain_costmap_plugin
{

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

unsigned char traversabilityToCost(
  float t, double free_threshold, double lethal_threshold, unsigned char max_non_lethal)
{
  if (!std::isfinite(t)) {
    return NO_INFORMATION;
  }
  if (t >= lethal_threshold - 1e-6) {
    return LETHAL_OBSTACLE;
  }
  if (t < free_threshold) {
    return FREE_SPACE;
  }
  const double ratio = (t - free_threshold) / (lethal_threshold - free_threshold);
  const double cost = 1.0 + ratio * (max_non_lethal - 1.0);
  return static_cast<unsigned char>(std::clamp(std::lround(cost), 1L, long{max_non_lethal}));
}

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/elevation_map")));
  declareParameter("layer_name", rclcpp::ParameterValue(std::string("traversability")));
  declareParameter("free_threshold", rclcpp::ParameterValue(0.3));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(0.95));
  declareParameter("max_non_lethal_cost", rclcpp::ParameterValue(252));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  declareParameter("use_maximum", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".layer_name", layer_name_);
  node->get_parameter(name_ + ".free_threshold", free_threshold_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".max_non_lethal_cost", max_non_lethal_cost_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".use_maximum", use_maximum_);
  max_non_lethal_cost_ = std::clamp(max_non_lethal_cost_, 1, 252);

  default_value_ = NO_INFORMATION;
  matchSize();
  current_ = true;

  sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&TraversabilityLayer::mapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(), "%s: '%s' layer from %s (free < %.2f, lethal >= %.2f)",
    name_.c_str(), layer_name_.c_str(), topic_.c_str(), free_threshold_, lethal_threshold_);
}

void TraversabilityLayer::mapCallback(grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = msg;
  new_map_ = true;
}

void TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double ,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }
  if (!enabled_) {
    return;
  }
  paintLatestMap(min_x, min_y, max_x, max_y);
}

void TraversabilityLayer::paintLatestMap(
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  grid_map_msgs::msg::GridMap::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!new_map_ || !latest_) {
      return;
    }
    msg = latest_;
    new_map_ = false;
  }

  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, map, {layer_name_}, false, false) ||
    !map.exists(layer_name_))
  {
    RCLCPP_WARN_ONCE(logger_, "%s: grid map has no '%s' layer", name_.c_str(), layer_name_.c_str());
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  tf2::Transform global_to_grid;
  try {
    const auto tf = tf_->lookupTransform(
      map.getFrameId(), global_frame, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
    tf2::fromMsg(tf.transform, global_to_grid);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "%s: TF %s -> %s: %s", name_.c_str(),
      global_frame.c_str(), map.getFrameId().c_str(), e.what());
    new_map_ = true;
    return;
  }
  const tf2::Transform grid_to_global = global_to_grid.inverse();

  const grid_map::Position c = map.getPosition();
  const double hx = map.getLength().x() / 2.0, hy = map.getLength().y() / 2.0;
  double bx0 = std::numeric_limits<double>::max(), by0 = bx0;
  double bx1 = std::numeric_limits<double>::lowest(), by1 = bx1;
  for (int sx : {-1, 1}) {
    for (int sy : {-1, 1}) {
      const tf2::Vector3 p = grid_to_global * tf2::Vector3(c.x() + sx * hx, c.y() + sy * hy, 0.0);
      bx0 = std::min(bx0, p.x()); by0 = std::min(by0, p.y());
      bx1 = std::max(bx1, p.x()); by1 = std::max(by1, p.y());
    }
  }

  int mx0, my0, mx1, my1;
  worldToMapEnforceBounds(bx0, by0, mx0, my0);
  worldToMapEnforceBounds(bx1, by1, mx1, my1);

  const grid_map::Matrix & layer = map[layer_name_];
  grid_map::Index idx;
  for (int my = my0; my <= my1; ++my) {
    for (int mx = mx0; mx <= mx1; ++mx) {
      double wx, wy;
      mapToWorld(mx, my, wx, wy);
      const tf2::Vector3 p = global_to_grid * tf2::Vector3(wx, wy, 0.0);
      if (!map.getIndex(grid_map::Position(p.x(), p.y()), idx)) {
        continue;
      }
      const unsigned char cost = traversabilityToCost(
        layer(idx(0), idx(1)), free_threshold_, lethal_threshold_,
        static_cast<unsigned char>(max_non_lethal_cost_));
      if (cost != NO_INFORMATION) {
        setCost(mx, my, cost);
      }
    }
  }

  *min_x = std::min(*min_x, bx0);
  *min_y = std::min(*min_y, by0);
  *max_x = std::max(*max_x, bx1);
  *max_y = std::max(*max_y, by1);
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  if (use_maximum_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
  }
}

void TraversabilityLayer::reset()
{
  resetMaps();
  std::lock_guard<std::mutex> lock(mutex_);
  new_map_ = latest_ != nullptr;
  current_ = true;
}

void TraversabilityLayer::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  new_map_ = latest_ != nullptr;
}

void TraversabilityLayer::deactivate() {}

}

PLUGINLIB_EXPORT_CLASS(terrain_costmap_plugin::TraversabilityLayer, nav2_costmap_2d::Layer)
