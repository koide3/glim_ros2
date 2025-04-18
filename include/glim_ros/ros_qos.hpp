#pragma once
#include <glim/util/config.hpp>
#include <rclcpp/rclcpp.hpp>

namespace glim {
    /**
     * @brief Get QoS settings from configuration file.
     * @param config_ros Config instance to read QoS settings from.
     * @param module_name Module name.
     * @param parameter_name Name of QoS parameter.
     */
    rclcpp::QoS get_qos_settings(const Config& config_ros, const std::string& module_name, const std::string& param_name);
}