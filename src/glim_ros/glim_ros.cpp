#include <glim_ros/glim_ros.hpp>

#include <deque>
#include <thread>
#include <iostream>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>

#include <glim/util/config.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/async_odometry_estimation.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/frontend/odometry_estimation_gpu.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>

#include <glim_ros/raw_points.hpp>

namespace glim_ros {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  const std::string config_path = ament_index_cpp::get_package_share_directory("glim") + "/config";
  auto global_config = glim::GlobalConfig::instance(config_path);

  std::cout << "config_path:" << config_path << std::endl;
  // global_config->override_param<std::string>("global", "config_path", "/home/koide/ros2_ws/src/glim/config");

  latest_imu_stamp = 0.0;

  standard_viewer.reset(new glim::StandardViewer);

  preprocessor.reset(new glim::CloudPreprocessor);
  odometry_estimation.reset(new glim::AsyncOdometryEstimation(std::shared_ptr<glim::OdometryEstimationGPU>(new glim::OdometryEstimationGPU)));
  sub_mapping.reset(new glim::AsyncSubMapping(std::shared_ptr<glim::SubMapping>(new glim::SubMapping)));
  global_mapping.reset(new glim::AsyncGlobalMapping(std::shared_ptr<glim::GlobalMapping>(new glim::GlobalMapping)));

  // const std::string points_topic = "/camera/depth/color/points";
  // const std::string imu_topic = "/camera/imu";
  const std::string imu_topic = "/boomtop/imu/front_right";
  const std::string points_topic = "/boomtop/lidar/front_right";
  const std::string image_topic = "/camera/color/image_raw";

  using std::placeholders::_1;
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 100, std::bind(&GlimROS::imu_callback, this, _1));
  // image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic, 10, std::bind(&GlimROS::image_callback, this, _1));
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, 10, std::bind(&GlimROS::points_callback, this, _1));
  // points_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>(points_topic, 10, std::bind(&GlimROS::livox_points_callback, this, _1));

  timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&GlimROS::timer_callback, this));
}

GlimROS::~GlimROS() {
  if (standard_viewer) {
    standard_viewer->stop();
  }
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const double imu_acc_scale_factor = 9.80665;

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  const Eigen::Vector3d linear_acc = imu_acc_scale_factor * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);

  latest_imu_stamp = imu_stamp;
}

void GlimROS::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
}

void GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  auto frame = glim_ros::RawPoints::extract(msg);
  if (frame == nullptr) {
    RCLCPP_WARN_STREAM(this->get_logger(), "failed to extract points from message");
    return;
  }

  const double scan_duration = 0.05;
  if (frame->times.empty()) {
    frame->times.resize(frame->points.size());
    for (int i = 0; i < frame->points.size(); i++) {
      frame->times[i] = scan_duration * static_cast<double>(i) / frame->points.size();
    }
  }

  frame_queue.push_back(frame);

  while (!frame_queue.empty() && frame_queue.front()->stamp + 0.1 < latest_imu_stamp) {
    const auto front = frame_queue.front();
    auto preprocessed = preprocessor->preprocess(front->stamp, front->times, front->points);
    odometry_estimation->insert_frame(preprocessed);
    frame_queue.pop_front();
  }
}

void GlimROS::livox_points_callback(const livox_interfaces::msg::CustomMsg::ConstSharedPtr msg) {
  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  std::vector<double> times(msg->point_num);
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points(msg->point_num);

  for (int i = 0; i < msg->point_num; i++) {
    const auto& point = msg->points[i];
    times[i] = point.offset_time / 1e9;
    points[i] << point.x, point.y, point.z, 1.0;
  }

  glim_ros::RawPoints::Ptr frame(new glim_ros::RawPoints);
  frame->stamp = stamp;
  frame->times = times;
  frame->points = points;

  frame_queue.push_back(frame);

  while (!frame_queue.empty() && frame_queue.front()->stamp + 0.1 < latest_imu_stamp) {
    const auto front = frame_queue.front();
    auto preprocessed = preprocessor->preprocess(front->stamp, front->times, front->points);
    odometry_estimation->insert_frame(preprocessed);
    frame_queue.pop_front();
  }
}

void GlimROS::timer_callback() {
  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  for (const auto& frame : marginalized_frames) {
    sub_mapping->insert_frame(frame);
  }

  auto submaps = sub_mapping->get_results();

  for (const auto& submap : submaps) {
    global_mapping->insert_submap(submap);
  }

  if (!standard_viewer->ok()) {
    rclcpp::shutdown();
  }
}

bool GlimROS::ok() const {
  if (!standard_viewer) {
    return true;
  }
  return standard_viewer->ok();
}

void GlimROS::wait() {
  if (standard_viewer) {
    standard_viewer->wait();
  }
}

void GlimROS::save(const std::string& path) {
  global_mapping->save(path);
}

}  // namespace glim_ros