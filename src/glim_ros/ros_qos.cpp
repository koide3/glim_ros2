#include <glim_ros/ros_qos.hpp>
#include <spdlog/spdlog.h>

namespace glim {

static bool init_qos_profile(const std::string& name, rmw_qos_profile_t& profile) {
  bool found = true;
  if (name == "default") {
    profile = rmw_qos_profile_default;
  } else if (name == "system_default") {
    profile = rmw_qos_profile_system_default;
  } else if (name == "sensor_data") {
    profile = rmw_qos_profile_sensor_data;
  } else if (name == "services_default") {
    profile = rmw_qos_profile_services_default;
  } else if (name == "parameters") {
    profile = rmw_qos_profile_parameters;
  } else if (name == "parameter_events") {
    profile = rmw_qos_profile_parameter_events;
  } else {
    profile = rmw_qos_profile_default;
    found = false;
  }
  return found;
}

static bool get_qos(const Config& config_ros, const std::string& module_name, const std::string& param_name, std::string& profile_name, rclcpp::QoS& qos) {
  bool is_configured = false;
  rmw_qos_profile_t profile;
  std::vector<std::string> module_path;

  module_path.push_back(module_name);
  module_path.push_back(param_name);

  auto profile_param = config_ros.param_nested<std::string>(module_path, "profile");
  if (profile_param.has_value()) {
    profile_name = profile_param.value();
    is_configured = true;
  } else {
    profile_name = "sensor_data";
  }

  if (!init_qos_profile(profile_name, profile)) {
    spdlog::warn("unknown QoS profile '{}', falling back to 'default'.", profile_name);
  }

  auto depth = config_ros.param_nested<int>(module_path, "depth");
  if (depth.has_value()) {
    profile.depth = depth.value();
    is_configured = true;
  }

  auto str = config_ros.param_nested<std::string>(module_path, "durability");
  if (str.has_value()) {
    auto value = rmw_qos_durability_policy_from_str(str.value().c_str());
    if (value == RMW_QOS_POLICY_DURABILITY_UNKNOWN) {
      spdlog::warn("ignoring unknown durability policy '{}'.", str.value());
    } else {
      profile.durability = value;
      is_configured = true;
    }
  }

  str = config_ros.param_nested<std::string>(module_path, "reliability");
  if (str.has_value()) {
    auto value = rmw_qos_reliability_policy_from_str(str.value().c_str());
    if (value == RMW_QOS_POLICY_RELIABILITY_UNKNOWN) {
      spdlog::warn("ignoring unknown reliability policy '{}'.", str.value());
    } else {
      profile.reliability = value;
      is_configured = true;
    }
  }

  str = config_ros.param_nested<std::string>(module_path, "history");
  if (str.has_value()) {
    auto value = rmw_qos_history_policy_from_str(str.value().c_str());
    if (value == RMW_QOS_POLICY_HISTORY_UNKNOWN) {
      spdlog::warn("ignoring unknown history policy '{}'.", str.value());
    } else {
      profile.history = value;
      is_configured = true;
    }
  }

  qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
  return is_configured;
}

rclcpp::QoS get_qos_settings(const Config& config_ros, const std::string& module_name, const std::string& param_name, const std::optional<rclcpp::QoS>& default_qos) {
  std::string profile_name;
  rclcpp::QoS qos{0};

  if (!get_qos(config_ros, module_name, param_name, profile_name, qos) && default_qos.has_value()) {
    qos = default_qos.value();
  }

  spdlog::trace(
    "{}: profile={}, depth={}, history={}, reliability={}, durability={}",
    param_name,
    profile_name,
    qos.depth(),
    rmw_qos_history_policy_to_str(qos.get_rmw_qos_profile().history),
    rmw_qos_reliability_policy_to_str(qos.get_rmw_qos_profile().reliability),
    rmw_qos_durability_policy_to_str(qos.get_rmw_qos_profile().durability));

  return qos;
}

}  // namespace glim