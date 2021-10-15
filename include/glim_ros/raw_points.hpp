#pragma once

#include <vector>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace glim_ros {

struct RawPoints {
public:
  using Ptr = std::shared_ptr<RawPoints>;
  using ConstPtr = std::shared_ptr<const RawPoints>;

  static RawPoints::Ptr extract(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points_msg);

public:
  double stamp;
  std::vector<double> times;
  std::vector<double> intensities;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
};
}