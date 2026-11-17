#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "terrain_nav_mapping/traversability.hpp"

namespace terrain_nav_mapping
{

class ElevationMapperNode : public rclcpp::Node
{
public:
  ElevationMapperNode()
  : Node("elevation_mapper")
  {
    map_frame_ = declare_parameter("map_frame", "odom");
    robot_frame_ = declare_parameter("robot_frame", "base_footprint");
    const double length = declare_parameter("map_length", 40.0);
    const double resolution = declare_parameter("resolution", 0.2);
    min_range_ = declare_parameter("min_range", 0.9);
    max_range_ = declare_parameter("max_range", 12.0);
    alpha_ = declare_parameter("ema_alpha", 0.5);
    const double publish_rate = declare_parameter("publish_rate", 2.0);

    params_.slope_crit_rad = declare_parameter("slope_crit_deg", 20.0) * M_PI / 180.0;
    params_.step_crit = declare_parameter("step_crit", 0.15);
    params_.roughness_crit = declare_parameter("roughness_crit", 0.08);
    params_.plane_window = declare_parameter("plane_window", 5);
    params_.step_window = declare_parameter("step_window", 3);
    params_.min_valid_cells = declare_parameter("min_valid_cells", 6);

    map_.setFrameId(map_frame_);
    map_.setGeometry(grid_map::Length(length, length), resolution);
    map_.add(layers::kElevation);
    map_.add("scan_max");
    RCLCPP_INFO(
      get_logger(), "Elevation map %.0f x %.0f m @ %.2f m in '%s'",
      length, length, resolution, map_frame_.c_str());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS(),
      std::bind(&ElevationMapperNode::cloudCallback, this, std::placeholders::_1));

    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
      "elevation_map", rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&ElevationMapperNode::publishMap, this));
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_cloud, tf_robot;
    try {
      tf_cloud = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
      tf_robot = tf_buffer_->lookupTransform(
        map_frame_, robot_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF: %s", e.what());
      return;
    }
    const Eigen::Isometry3d T = tf2::transformToEigen(tf_cloud);

    map_.move(grid_map::Position(
        tf_robot.transform.translation.x, tf_robot.transform.translation.y));

    grid_map::Matrix & scan_max = map_["scan_max"];
    scan_max.setConstant(std::numeric_limits<float>::quiet_NaN());

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x"), it_y(*msg, "y"),
    it_z(*msg, "z");
    grid_map::Index idx;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const Eigen::Vector3d p_s(*it_x, *it_y, *it_z);
      if (!p_s.allFinite()) {continue;}
      const double range = std::hypot(p_s.x(), p_s.y());
      if (range < min_range_ || range > max_range_) {continue;}

      const Eigen::Vector3d p = T * p_s;
      if (!map_.getIndex(grid_map::Position(p.x(), p.y()), idx)) {continue;}
      float & cell = scan_max(idx(0), idx(1));
      if (!std::isfinite(cell) || p.z() > cell) {
        cell = static_cast<float>(p.z());
      }
    }

    grid_map::Matrix & elev = map_[layers::kElevation];
    for (Eigen::Index i = 0; i < elev.size(); ++i) {
      const float m = scan_max(i);
      if (!std::isfinite(m)) {continue;}
      float & e = elev(i);
      e = std::isfinite(e) ? static_cast<float>(e + alpha_ * (m - e)) : m;
    }
    last_stamp_ = msg->header.stamp;
    have_data_ = true;
  }

  void publishMap()
  {
    if (!have_data_) {return;}
    computeTraversability(map_, params_);
    map_.setTimestamp(rclcpp::Time(last_stamp_).nanoseconds());

    auto msg = grid_map::GridMapRosConverter::toMessage(
      map_, {layers::kElevation, layers::kSlope, layers::kRoughness, layers::kStep,
        layers::kTraversability});
    map_pub_->publish(std::move(msg));
  }

  std::string map_frame_, robot_frame_;
  double min_range_, max_range_, alpha_;
  TraversabilityParams params_;

  grid_map::GridMap map_;
  builtin_interfaces::msg::Time last_stamp_;
  bool have_data_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<terrain_nav_mapping::ElevationMapperNode>());
  rclcpp::shutdown();
  return 0;
}
