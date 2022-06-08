#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

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

#include <glim/util/config.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/async_odometry_estimation.hpp>
#include <glim/frontend/odometry_estimation_ct.hpp>
#include <glim/frontend/odometry_estimation_gpu.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  const std::string config_ros_path = ament_index_cpp::get_package_share_directory("glim_ros") + "/config/glim_ros.json";
  std::cout << "config_ros_path:" << config_ros_path << std::endl;
  glim::Config config_ros(config_ros_path);

  const std::string config_path = ament_index_cpp::get_package_share_directory("glim") + config_ros.param<std::string>("glim_ros", "config_path", "/config");
  std::cout << "config_path:" << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);

  latest_imu_stamp = 0.0;

  // Viewer
  if (config_ros.param<bool>("glim_ros", "enable_viewer", true)) {
    standard_viewer.reset(new glim::StandardViewer);
  }

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_front(glim::GlobalConfig::get_config_path("config_frontend"));
  const std::string frontend_mode = config_front.param<std::string>("odometry_estimation", "frontend_mode", "CPU");

  std::shared_ptr<glim::OdometryEstimationBase> odom;
  if (frontend_mode == "CPU") {
    RCLCPP_WARN_STREAM(this->get_logger(), "CPU frontend has not been implemented yet!!");
  } else if (frontend_mode == "GPU") {
#ifdef BUILD_GTSAM_EXT_GPU
    odom.reset(new glim::OdometryEstimationGPU);
#else
    RCLCPP_WARN_STREAM(this->get_logger(), "GPU frontend is selected although glim was built without GPU support!!");
#endif
  } else if (frontend_mode == "CT") {
    odom.reset(new glim::OdometryEstimationCT);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown frontend mode:" << frontend_mode);
    abort();
  }

  odometry_estimation = odom;

  // Backend
  sub_mapping.reset(new glim::AsyncSubMapping(std::shared_ptr<glim::SubMapping>(new glim::SubMapping)));
  global_mapping.reset(new glim::AsyncGlobalMapping(std::shared_ptr<glim::GlobalMapping>(new glim::GlobalMapping)));

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    std::cout << console::bold_red << "Extension modules are enabled!!" << console::reset << std::endl;
    std::cout << console::bold_red << "You must carefully check and follow the licenses of ext modules" << console::reset << std::endl;

    const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
    std::cout << "config_ext_path: " << config_ext_path << std::endl;
    glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);

    for (const auto& extension : *extensions) {
      auto ext_module = ExtensionModule::load(extension);
      if (ext_module == nullptr) {
        std::cerr << console::bold_red << "error: failed to load " << extension << console::reset << std::endl;
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions();
          extension_subs.insert(extension_subs.end(), subs.begin(), subs.end());
        }
      }
    }
  }

  timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&GlimROS::timer_callback, this));
}

GlimROS::~GlimROS() {
  if (standard_viewer) {
    standard_viewer->stop();
  }
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  const double imu_acc_scale_factor = 9.80665;

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  const Eigen::Vector3d linear_acc = imu_acc_scale_factor * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  time_keeper->validate_imu_stamp(imu_stamp);

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);

  latest_imu_stamp = imu_stamp;
}

void GlimROS::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");
}

void GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  auto raw_points = glim::extract_raw_points(msg);
  if (raw_points == nullptr) {
    RCLCPP_WARN_STREAM(this->get_logger(), "failed to extract points from message");
    return;
  }

  time_keeper->process(raw_points);
  frame_queue.push_back(raw_points);

  while (!frame_queue.empty() && frame_queue.front()->stamp + 0.1 < latest_imu_stamp) {
    const auto front = frame_queue.front();
    auto preprocessed = preprocessor->preprocess(front);

    for (double& intensity : preprocessed->intensities) {
      intensity /= 128.0;
    }

    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->insert_frame(preprocessed, marginalized_frames);
    frame_queue.pop_front();
  }
}

#ifdef BUILD_WITH_LIVOX
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
#endif

void GlimROS::timer_callback() {
  if (!standard_viewer->ok()) {
    rclcpp::shutdown();
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  // odometry_estimation->get_results(estimation_frames, marginalized_frames);

  return;

  for (const auto& frame : marginalized_frames) {
    sub_mapping->insert_frame(frame);
  }

  auto submaps = sub_mapping->get_results();
  for (const auto& submap : submaps) {
    global_mapping->insert_submap(submap);
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
  // global_mapping->save(path);
}

}  // namespace glim