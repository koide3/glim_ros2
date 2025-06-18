#include <glim_ros/glim_ros.hpp>

#define GLIM_ROS2

#include <deque>
#include <thread>
#include <iostream>
#include <functional>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <boost/format.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex> // For std::lock_guard in initial_pose_callback

// GTSAM related includes
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
// Note: <gtsam/base/numericalDerivative.h> and <gtsam/base/serialization.h> are not strictly needed for Diagonal noise model.

#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>

#include <glim/util/debug.hpp>
#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/mapping/async_sub_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim_ros/ros_compatibility.hpp>
#include <glim_ros/ros_qos.hpp>

namespace glim {

// Helper function to format rclcpp::Time
std::string format_ros_time(const rclcpp::Time& stamp) {
    std::chrono::seconds secs(stamp.seconds());
    std::chrono::nanoseconds nsecs(stamp.nanoseconds());

    auto total_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(secs + nsecs);
    auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(total_nsecs);
    auto tt = std::chrono::system_clock::to_time_t(tp);

    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&tt), "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(9) << nsecs.count();
    return oss.str();
}

GlimROS::GlimROS(const rclcpp::NodeOptions& options) : Node("glim_ros", options) {
  // Setup logger
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(get_ringbuffer_sink());
  spdlog::set_default_logger(logger);

  bool debug = false;
  this->declare_parameter<bool>("debug", false);
  this->get_parameter<bool>("debug", debug);

  if (debug) {
    spdlog::info("enable debug printing");
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("/tmp/glim_log.log", true);
    logger->sinks().push_back(file_sink);
    logger->set_level(spdlog::level::trace);

    print_system_info(logger);
  }

  dump_on_unload = false;
  this->declare_parameter<bool>("dump_on_unload", false);
  this->get_parameter<bool>("dump_on_unload", dump_on_unload);

  if (dump_on_unload) {
    spdlog::info("dump_on_unload={}", dump_on_unload);
  }

  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  logger->info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  // Declare and retrieve initial pose parameters
  this->declare_parameter<double>("initial_pose.position.x", 0.0);
  this->declare_parameter<double>("initial_pose.position.y", 0.0);
  this->declare_parameter<double>("initial_pose.position.z", 0.0);
  this->declare_parameter<double>("initial_pose.orientation.x", 0.0);
  this->declare_parameter<double>("initial_pose.orientation.y", 0.0);
  this->declare_parameter<double>("initial_pose.orientation.z", 0.0);
  this->declare_parameter<double>("initial_pose.orientation.w", 1.0);

  this->get_parameter("initial_pose.position.x", initial_pose_position_x_);
  this->get_parameter("initial_pose.position.y", initial_pose_position_y_);
  this->get_parameter("initial_pose.position.z", initial_pose_position_z_);
  this->get_parameter("initial_pose.orientation.x", initial_pose_orientation_x_);
  this->get_parameter("initial_pose.orientation.y", initial_pose_orientation_y_);
  this->get_parameter("initial_pose.orientation.z", initial_pose_orientation_z_);
  this->get_parameter("initial_pose.orientation.w", initial_pose_orientation_w_);

  if (initial_pose_position_x_ != 0.0 ||
      initial_pose_position_y_ != 0.0 ||
      initial_pose_position_z_ != 0.0 ||
      initial_pose_orientation_x_ != 0.0 ||
      initial_pose_orientation_y_ != 0.0 ||
      initial_pose_orientation_z_ != 0.0 ||
      initial_pose_orientation_w_ != 1.0) {
    has_initial_pose_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Initial pose loaded from parameters: P[%.3f, %.3f, %.3f], O[%.3f, %.3f, %.3f, %.3f]",
      initial_pose_position_x_,
      initial_pose_position_y_,
      initial_pose_position_z_,
      initial_pose_orientation_x_,
      initial_pose_orientation_y_,
      initial_pose_orientation_z_,
      initial_pose_orientation_w_);
  } else {
    RCLCPP_INFO(this->get_logger(), "No initial pose provided via parameters, or it's the default pose [0,0,0], [0,0,0,1].");
  }

  // Declare and retrieve map_load_path_on_start parameter
  this->declare_parameter<std::string>("map_load_path_on_start", "");
  this->get_parameter("map_load_path_on_start", map_load_path_on_start_);

  if (!map_load_path_on_start_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Parameter 'map_load_path_on_start' set to: %s", map_load_path_on_start_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Parameter 'map_load_path_on_start' is not set. Map will not be loaded on startup by this parameter.");
  }

  keep_raw_points = config_ros.param<bool>("glim_ros", "keep_raw_points", false);
  imu_time_offset = config_ros.param<double>("glim_ros", "imu_time_offset", 0.0);
  points_time_offset = config_ros.param<double>("glim_ros", "points_time_offset", 0.0);
  acc_scale = config_ros.param<double>("glim_ros", "acc_scale", 1.0);

  glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
  intensity_field = config_sensors.param<std::string>("sensors", "intensity_field", "intensity");
  ring_field = config_sensors.param<std::string>("sensors", "ring_field", "");

  // Setup GPU-based linearization
#ifdef BUILD_GTSAM_POINTS_GPU
  gtsam_points::LinearizationHook::register_hook([]() { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

  initialize_core_slam_modules();

  // Extention modules
  const auto extensions = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules");
  if (extensions && !extensions->empty()) {
    for (const auto& extension : *extensions) {
      if (extension.find("viewer") == std::string::npos && extension.find("monitor") == std::string::npos) {
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
      auto ext_module = ExtensionModule::load_module(extension);
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

  // Subscribers
  rclcpp::SensorDataQoS default_imu_qos;
  default_imu_qos.get_rmw_qos_profile().depth = 1000;
  auto qos = get_qos_settings(config_ros, "glim_ros", "imu_qos", default_imu_qos);
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos, std::bind(&GlimROS::imu_callback, this, _1));

  qos = get_qos_settings(config_ros, "glim_ros", "points_qos");
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, qos, std::bind(&GlimROS::points_callback, this, _1));
#ifdef BUILD_WITH_CV_BRIDGE
  qos = get_qos_settings(config_ros, "glim_ros", "image_qos");
  image_sub = image_transport::create_subscription(this, image_topic, std::bind(&GlimROS::image_callback, this, _1), "raw", qos.get_rmw_qos_profile());
#endif

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&GlimROS::initial_pose_callback, this, _1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /initialpose topic for initial pose updates.");

  for (const auto& sub : this->extension_subscriptions()) {
    spdlog::debug("subscribe to {}", sub->topic);
    sub->create_subscriber(*this);
  }

  // Start timer
  timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { timer_callback(); });

  // Request map loading on startup if the path is provided
  if (!map_load_path_on_start_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Requesting map load on startup for path: %s", map_load_path_on_start_.c_str());
    std::lock_guard<std::mutex> lock(map_load_mutex_);
    requested_map_path_ = map_load_path_on_start_;
    map_load_requested_ = true;
  }

  spdlog::debug("initialized");
}

GlimROS::~GlimROS() {
  spdlog::debug("quit");
  extension_modules.clear();

  if (dump_on_unload) {
    std::string dump_path = "/tmp/dump";
    wait(true);
    save(dump_path);
  }
}

void GlimROS::request_map_load_from_gui(const std::string& map_path) {
  std::lock_guard<std::mutex> lock(map_load_mutex_);
  map_load_requested_ = true;
  requested_map_path_ = map_path;
  spdlog::info("Map load requested from GUI: {}", map_path); // Optional: for logging
}

void GlimROS::initialize_core_slam_modules() {
    spdlog::info("Initializing core SLAM modules...");

    // TimeKeeper and Preprocessor initialization
    time_keeper.reset(new glim::TimeKeeper);
    preprocessor.reset(new glim::CloudPreprocessor);
    spdlog::debug("TimeKeeper and CloudPreprocessor initialized.");

    // Odometry estimation initialization
    glim::Config config_odometry(glim::GlobalConfig::get_config_path("config_odometry"));
    const std::string odometry_estimation_so_name = config_odometry.param<std::string>("odometry_estimation", "so_name", "libodometry_estimation_cpu.so");
    spdlog::info("load {}", odometry_estimation_so_name);

    std::shared_ptr<glim::OdometryEstimationBase> odom = OdometryEstimationBase::load_module(odometry_estimation_so_name);
    if (!odom) {
        spdlog::critical("failed to load odometry estimation module");
        // Consider how to handle this error. Throwing an exception might be appropriate
        // or setting an error state that the calling code can check.
        // For now, following existing pattern of abort().
        abort();
    }
    odometry_estimation.reset(new glim::AsyncOdometryEstimation(odom, odom->requires_imu()));
    spdlog::debug("Odometry estimation module initialized.");

    // Sub mapping initialization
    // Retain existing logic for checking "enable_local_mapping"
    glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros")); // Assuming config_ros is needed here, or pass it if it's local to constructor
    if (config_ros.param<bool>("glim_ros", "enable_local_mapping", true)) {
        const std::string sub_mapping_so_name =
          glim::Config(glim::GlobalConfig::get_config_path("config_sub_mapping")).param<std::string>("sub_mapping", "so_name", "libsub_mapping.so");
        if (!sub_mapping_so_name.empty()) {
            spdlog::info("load {}", sub_mapping_so_name);
            auto sub = SubMappingBase::load_module(sub_mapping_so_name);
            if (sub) {
                sub_mapping.reset(new AsyncSubMapping(sub));
                spdlog::debug("Sub mapping module initialized.");
            } else {
                spdlog::warn("Failed to load sub mapping module: {}", sub_mapping_so_name);
            }
        }
    } else {
        spdlog::info("Local mapping is disabled.");
    }

    // Global mapping initialization
    // Retain existing logic for checking "enable_global_mapping"
    if (config_ros.param<bool>("glim_ros", "enable_global_mapping", true)) {
        const std::string global_mapping_so_name =
          glim::Config(glim::GlobalConfig::get_config_path("config_global_mapping")).param<std::string>("global_mapping", "so_name", "libglobal_mapping.so");
        if (!global_mapping_so_name.empty()) {
            spdlog::info("load {}", global_mapping_so_name);
            auto global = GlobalMappingBase::load_module(global_mapping_so_name);
            if (global) {
                global_mapping.reset(new AsyncGlobalMapping(global));
                spdlog::debug("Global mapping module initialized.");
            } else {
                spdlog::warn("Failed to load global mapping module: {}", global_mapping_so_name);
            }
        }
    } else {
        spdlog::info("Global mapping is disabled.");
    }
    spdlog::info("Core SLAM modules initialization complete.");
}

void GlimROS::shutdown_core_slam_modules() {
  spdlog::info("Shutting down core SLAM modules...");

  if (odometry_estimation) {
    odometry_estimation->join(); // Ensure threads are stopped before reset
    odometry_estimation.reset();
    spdlog::debug("Odometry estimation module reset.");
  }
  if (sub_mapping) {
    sub_mapping->join(); // Ensure threads are stopped before reset
    sub_mapping.reset();
    spdlog::debug("Sub mapping module reset.");
  }
  if (global_mapping) {
    global_mapping->join(); // Ensure threads are stopped before reset
    global_mapping.reset();
    spdlog::debug("Global mapping module reset.");
  }

  spdlog::info("Core SLAM modules shutdown complete.");
}

const std::vector<std::shared_ptr<GenericTopicSubscription>>& GlimROS::extension_subscriptions() {
  return extension_subs;
}

void GlimROS::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  spdlog::trace("IMU: {}", format_ros_time(msg->header.stamp));

  const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9 + imu_time_offset;
  const Eigen::Vector3d linear_acc = acc_scale * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  if (!time_keeper->validate_imu_stamp(imu_stamp)) {
    spdlog::warn("skip an invalid IMU data (stamp={})", imu_stamp);
    return;
  }

  odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
  if (sub_mapping) {
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
  if (global_mapping) {
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
  }
}

#ifdef BUILD_WITH_CV_BRIDGE
void GlimROS::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  spdlog::trace("image: {}", format_ros_time(msg->header.stamp));

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
#endif

size_t GlimROS::points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  spdlog::trace("points: {}", format_ros_time(msg->header.stamp));

  auto raw_points = glim::extract_raw_points(*msg, intensity_field, ring_field);
  if (raw_points == nullptr) {
    spdlog::warn("failed to extract points from message");
    return 0;
  }

  raw_points->stamp += points_time_offset;
  time_keeper->process(raw_points);
  auto preprocessed = preprocessor->preprocess(raw_points);

  if (keep_raw_points) {
    // note: Raw points are used only in extension modules for visualization purposes.
    //       If you need to reduce the memory footprint, you can safely comment out the following line.
    preprocessed->raw_points = raw_points;
  }

  odometry_estimation->insert_frame(preprocessed);

  const size_t workload = odometry_estimation->workload();
  spdlog::debug("workload={}", workload);

  return workload;
}

void GlimROS::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(initial_pose_mutex_); // Protect access to shared pose variables

  initial_pose_position_x_ = msg->pose.pose.position.x;
  initial_pose_position_y_ = msg->pose.pose.position.y;
  initial_pose_position_z_ = msg->pose.pose.position.z;

  initial_pose_orientation_x_ = msg->pose.pose.orientation.x;
  initial_pose_orientation_y_ = msg->pose.pose.orientation.y;
  initial_pose_orientation_z_ = msg->pose.pose.orientation.z;
  initial_pose_orientation_w_ = msg->pose.pose.orientation.w;

  has_initial_pose_ = true;

  RCLCPP_INFO(this->get_logger(),
                "Received initial pose from /initialpose topic: P[%.3f, %.3f, %.3f], O[%.3f, %.3f, %.3f, %.3f]",
                initial_pose_position_x_, initial_pose_position_y_, initial_pose_position_z_,
                initial_pose_orientation_x_, initial_pose_orientation_y_, initial_pose_orientation_z_, initial_pose_orientation_w_);
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
  // Check for map load request
  std::string path_to_load;
  bool load_map = false;
  {
    std::lock_guard<std::mutex> lock(map_load_mutex_);
    if (map_load_requested_) {
      load_map = true;
      path_to_load = requested_map_path_;
      map_load_requested_ = false;
      requested_map_path_.clear();
    }
  }

  if (load_map) {
    spdlog::info("Map load request detected in timer_callback for path: {}", path_to_load);

    // 1. Shutdown existing SLAM modules
    shutdown_core_slam_modules();

    // 2. Re-initialize SLAM pipeline
    initialize_core_slam_modules();

    // 3. Load map into GlobalMapping
    if (global_mapping) { // This checks if AsyncGlobalMapping unique_ptr is valid
      // Check if global_mapping was actually initialized (e.g. not disabled by config)
      RCLCPP_INFO(this->get_logger(), "Loading map into GlobalMapping module from path: %s", path_to_load.c_str());

      std::shared_ptr<glim::GlobalMappingBase> base_mapping_ptr = global_mapping->get_global_mapping();
      if (base_mapping_ptr) {
        std::shared_ptr<glim::GlobalMapping> derived_mapping_ptr = std::dynamic_pointer_cast<glim::GlobalMapping>(base_mapping_ptr);
        if (derived_mapping_ptr) {
          if (derived_mapping_ptr->load(path_to_load)) {
            RCLCPP_INFO(this->get_logger(), "Map successfully loaded into GlobalMapping.");

            // Apply initial pose if available
            std::lock_guard<std::mutex> lock(initial_pose_mutex_);
            if (has_initial_pose_) {
                RCLCPP_INFO(this->get_logger(), "Applying initial pose to GlobalMapping.");
                gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(
                    initial_pose_orientation_w_,
                    initial_pose_orientation_x_,
                    initial_pose_orientation_y_,
                    initial_pose_orientation_z_
                );
                gtsam::Point3 position(
                    initial_pose_position_x_,
                    initial_pose_position_y_,
                    initial_pose_position_z_
                );
                gtsam::Pose3 initial_gtsam_pose(rotation, position);

                gtsam::Vector6 sigmas;
                sigmas << 0.1, 0.1, 0.1, // roll, pitch, yaw stddev in radians (adjust as needed)
                          0.2, 0.2, 0.2; // x, y, z stddev in meters (adjust as needed)
                gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

                // Assuming derived_mapping_ptr is std::shared_ptr<glim::GlobalMapping>
                // and it has the set_initial_pose_prior method.
                // TODO: Uncomment the following lines once set_initial_pose_prior is implemented in glim::GlobalMapping (core GLIM library)
                derived_mapping_ptr->set_initial_pose_prior(initial_gtsam_pose, noise_model);
                RCLCPP_INFO(this->get_logger(), "Initial pose prior factor added to GlobalMapping.");

                // Optionally reset has_initial_pose_ if it's a one-time operation for this map load
                // has_initial_pose_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "No initial pose to apply after map load.");
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map using GlobalMapping::load from path: %s", path_to_load.c_str());
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to dynamically cast GlobalMappingBase to GlobalMapping. Cannot load map. Type mismatch or RTTI issue?");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "AsyncGlobalMapping returned a null GlobalMappingBase pointer. Cannot load map.");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "GlobalMapping module (AsyncGlobalMapping wrapper) is not available/initialized. Cannot load map.");
    }
    RCLCPP_INFO(this->get_logger(), "Map load process completed in timer_callback.");
  }

  // Existing timer_callback logic starts here
  for (const auto& ext_module : extension_modules) {
    if (!ext_module->ok()) {
      rclcpp::shutdown();
      return; // Return early if shutting down
    }
  }

  std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
  std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
  // Check if odometry_estimation is valid before calling get_results, as it might have been reset
  if(odometry_estimation) {
    odometry_estimation->get_results(estimation_frames, marginalized_frames);
  } else {
    spdlog::warn("Odometry estimation module not available in timer_callback. Skipping odometry processing.");
    // If odometry is not available (e.g. after a failed map load and reinit),
    // we might not want to proceed with the rest of the callback logic that depends on its output.
    // However, the original code doesn't explicitly stop here, so we'll maintain that behavior.
  }


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
  if (global_mapping) global_mapping->save(path);
  for (auto& module : extension_modules) {
    module->at_exit(path);
  }
}

}  // namespace glim

RCLCPP_COMPONENTS_REGISTER_NODE(glim::GlimROS);