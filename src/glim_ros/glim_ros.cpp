#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <thread>
#include <iostream>
#include <functional>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
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

namespace glim {

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  bool debug = false;
  this->declare_parameter<bool>("debug", false);
  this->get_parameter<bool>("debug", debug);

  if (debug) {
    spdlog::info("enable debug printing");
    auto logger = spdlog::default_logger();
    logger->set_level(spdlog::level::trace);

    if (!logger->sinks().empty()) {
      auto console_sink = logger->sinks()[0];
      console_sink->set_level(spdlog::level::debug);
    }

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/glim_log.log", true);
    file_sink->set_level(spdlog::level::trace);
    logger->sinks().push_back(file_sink);

    // Share the default logger across libraries
    // Not sure if this is a correct way
    glim::set_default_logger(logger);
  }

  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  spdlog::info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 1.0);

  // Preprocessing
  time_keeper.reset(new glim::TimeKeeper);
  preprocessor.reset(new glim::CloudPreprocessor);

  // Odometry estimation
  glim::Config config_front(glim::GlobalConfig::get_config_path("config_frontend"));
  const std::string frontend_mode = config_front.param<std::string>("odometry_estimation", "frontend_mode", "CPU");

  bool enable_imu = true;
  std::shared_ptr<glim::OdometryEstimationBase> odom;
  if (frontend_mode == "CPU") {
    spdlog::info("use CPU-based Range-IMU odometry estimation");
    odom.reset(new glim::OdometryEstimationCPU);
  } else if (frontend_mode == "GPU") {
    spdlog::info("use GPU-based Range-IMU odometry estimation");
#ifdef BUILD_GTSAM_EXT_GPU
    odom.reset(new glim::OdometryEstimationGPU);
#else
    RCLCPP_WARN_STREAM(this->get_logger(), "GPU frontend is selected although glim was built without GPU support!!");
#endif
  } else if (frontend_mode == "CT") {
    spdlog::info("use CPU-based CT-ICP odometry estimation");
    enable_imu = false;
    odom.reset(new glim::OdometryEstimationCT);
  } else {
    spdlog::critical("unknown odometry estimation mode: {}", frontend_mode);
    abort();
  }

  odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, enable_imu));

  // Backend
  if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
    spdlog::info("enable local mapping");
    sub_mapping.reset(new glim::AsyncSubMapping(std::shared_ptr<glim::SubMapping>(new glim::SubMapping)));
    if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
      spdlog::info("enable global mapping");
      global_mapping.reset(new glim::AsyncGlobalMapping(std::shared_ptr<glim::GlobalMapping>(new glim::GlobalMapping)));
    }
  }

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos) {
        spdlog::warn("Extension modules are enabled!!");
        spdlog::warn("You must carefully check and follow the licenses of ext modules");

        try {
          const std::string config_ext_path = ament_index_cpp::get_package_share_directory("glim_ext") + "/config";
          spdlog::info("config_ext_path: {}", config_ext_path);
          glim::GlobalConfig::instance()->override_param<std::string>("global", "config_ext", config_ext_path);
        } catch (ament_index_cpp::PackageNotFoundError& e) {
          spdlog::warn("glim_ext package path was not found!!");
        }

        break;
      }
    }

    for (const auto& extension : *extensions) {
      spdlog::info("load {}", extension);
      auto ext_module = ExtensionModule::load(extension);
      if (ext_module == nullptr) {
        spdlog::error("failed to load {}", extension);
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

  // ROS-related
  using std::placeholders::_1;
  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

  timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });

  auto imu_qos = rclcpp::SensorDataQoS();
  imu_qos.get_rmw_qos_profile().depth = 100;

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, std::bind(&GlimROS::imu_callback, this, _1));
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, rclcpp::SensorDataQoS(), std::bind(&GlimROS::points_callback, this, _1));
  image_sub = image_transport::create_subscription(this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw", rmw_qos_profile_sensor_data);

  for (const auto& sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }
}

GlimROS::~GlimROS() {
  spdlog::debug("quit");
  extension_modules.clear();
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

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
  spdlog::trace("image: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  auto cv_image = cv_bridge::toCvCopy(msg, "bgr8");

  const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  odometry_estimation->insert_image(stamp, cv_image->image);
  if (sub_mapping) {
    sub_mapping->insert_image(stamp, cv_image->image);
  }
  if (global_mapping) {
    global_mapping->insert_image(stamp, cv_image->image);
  }
}

void GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}.{}", msg->header.stamp.sec, msg->header.stamp.nanosec);

  auto raw_points = glim::extract_raw_points(msg);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
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

bool GlimROS::needs_wait() {
  for (const auto& ext_module : extension_modules) {
    if (ext_module->needs_wait()) {
      return true;
    }
  }

  return false;
}

void GlimROS::timer_callback() {
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      rclcpp::shutdown();
    }
  }

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

void GlimROS::wait(bool auto_quit) {
  spdlog::info("waiting for odometry estimation");
  odometry_estimation->join();

  if (sub_mapping) {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_results;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_results, marginalized_frames);
    for (const auto& marginalized_frame : marginalized_frames) {
      sub_mapping->insert_frame(marginalized_frame);
    }

    spdlog::info("waiting for local mapping");
    sub_mapping->join();

    const auto submaps = sub_mapping->get_results();
    if (global_mapping) {
      for (const auto& submap : submaps) {
        global_mapping->insert_submap(submap);
      }
      spdlog::info("waiting for global mapping");
      global_mapping->join();
    }
  }

  if (!auto_quit) {
    bool terminate = false;
    while (!terminate && rclcpp::ok()) {
      for (const auto& ext_module : extension_modules) {
        terminate |= (!ext_module->ok());
      }
    }
  }
}

void GlimROS::save(const std::string& path) {
  global_mapping->save(path);
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);