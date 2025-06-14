#pragma once

#include <any>
#include <deque>
#include <memory>
#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void wait(bool auto_quit = false);
  void save(const std::string& path);
  void request_map_load_from_gui(const std::string& map_path);

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

private:
  // Members for map loading
  std::atomic<bool> map_load_requested_{false};
  std::string requested_map_path_;
  std::string map_load_path_on_start_; // Path to the map to load on startup
  std::mutex map_load_mutex_;

  // Initial pose members
  double initial_pose_position_x_ = 0.0;
  double initial_pose_position_y_ = 0.0;
  double initial_pose_position_z_ = 0.0;
  double initial_pose_orientation_x_ = 0.0;
  double initial_pose_orientation_y_ = 0.0;
  double initial_pose_orientation_z_ = 0.0;
  double initial_pose_orientation_w_ = 1.0;
  bool has_initial_pose_ = false;
  std::mutex initial_pose_mutex_;

  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  bool dump_on_unload;

  std::string intensity_field, ring_field;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  void initialize_core_slam_modules();
  void shutdown_core_slam_modules();

  // ROS-related
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
