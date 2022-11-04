#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <thread>
#include <iostream>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
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
#include <glim/frontend/odometry_estimation_cpu.hpp>
#include <glim/frontend/odometry_estimation_gpu.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/standard_viewer.hpp>
#include <glim_ros/rviz_viewer.hpp>

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  std::cout << "config_path: " << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 1.0);

  // Viewer
  if (config_ros.param<bool>("glim_ros", "enable_viewer", true)) {
    standard_viewer.reset(new glim::StandardViewer);
  }

  if (config_ros.param<bool>("glim_ros", "enable_rviz", true)) {
    extension_modules.push_back(std::make_shared<RvizViewer>(*this));
  }

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_front(glim::GlobalConfig::get_config_path("config_frontend"));
  const std::string frontend_mode = config_front.param<std::string>("odometry_estimation", "frontend_mode", "CPU");

  bool enable_imu = true;
  std::shared_ptr<glim::OdometryEstimationBase> odom;
  if (frontend_mode == "CPU") {
    odom.reset(new glim::OdometryEstimationCPU);
  } else if (frontend_mode == "GPU") {
#ifdef BUILD_GTSAM_EXT_GPU
    odom.reset(new glim::OdometryEstimationGPU);
#else
    RCLCPP_WARN_STREAM(this->get_logger(), "GPU frontend is selected although glim was built without GPU support!!");
#endif
  } else if (frontend_mode == "CT") {
    enable_imu = false;
    odom.reset(new glim::OdometryEstimationCT);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown frontend mode:" << frontend_mode);
    abort();
  }

  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, enable_imu));

  // Backend
  if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
    sub_mapping.reset(new glim::AsyncSubMapping(std::shared_ptr<glim::SubMapping>(new glim::SubMapping)));
    if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
      global_mapping.reset(new glim::AsyncGlobalMapping(std::shared_ptr<glim::GlobalMapping>(new glim::GlobalMapping)));
    }
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    std::cout << console::bold_red << "Extension modules are enabled!!" << console::reset << std::endl;
    std::cout << console::bold_red << "You must carefully check and follow the licenses of ext modules" << console::reset << std::endl;

    try {
      const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
      std::cout << "config_ext_path: " << config_ext_path << std::endl;
      glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
    } catch (ament_index_cpp::PackageNotFoundError& e) {
      std::cerr << console::yellow << "warning: glim_ext package path was not found!!" << console::reset << std::endl;
    }

    for (const auto& extension : *extensions) {
      auto ext_module = ExtensionModule::load(extension);
      if (ext_module == nullptr) {
        std::cerr << console::bold_red << "error: failed to load " << extension << console::reset << std::endl;
        continue;
      } else {
        extension_modules.push_back(ext_module);

        auto ext_module_ros = std::dynamic_pointer_cast<ExtensionModuleROS2>(ext_module);
        if (ext_module_ros) {
          const auto subs = ext_module_ros->create_subscriptions(*this);
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
  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc = acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  time_keeper->validate_imu_stamp(imu_stamp);

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
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
  auto preprocessed = preprocessor->preprocess(raw_points);

  // note: Raw points are used only in extension modules for visualization purposes.
  //       If you need to reduce the memory footprint, you can safely comment out the following line.
  preprocessed->raw_points = raw_points;

  while (odometry_estimation->input_queue_size() > 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  odometry_estimation->insert_frame(preprocessed);
}

void GlimROS::timer_callback() {
#ifdef BUILD_WITH_VIEWER
  if (!standard_viewer->ok()) {
    rclcpp::shutdown();
  }
#endif

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  odometry_estimation->get_results(estimation_frames, marginalized_frames);

  if (sub_mapping) {
    for (const auto& frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
    }
  }
}

bool GlimROS::ok() const {
#ifdef BUILD_WITH_VIEWER
  if (!standard_viewer) {
    return rclcpp::ok();
  }
  return standard_viewer->ok() && rclcpp::ok();
#else
  return rclcpp::ok();
#endif
}

void GlimROS::wait(bool auto_quit) {
  std::cout << "odometry" << std::endl;
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    std::cout << "submap" << std::endl;
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      global_mapping->join();
    }
  }

#ifdef BUILD_WITH_VIEWER
  if (!auto_quit) {
    if (standard_viewer && rclcpp::ok()) {
      standard_viewer->wait();
    }
  } else {
    if (standard_viewer) {
      standard_viewer->stop();
    }
  }
#endif
}

void GlimROS::save(const std::string& path) {
  global_mapping->save(path);
}

}  // namespace glim