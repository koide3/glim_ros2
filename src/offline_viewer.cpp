#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/viewer/offline_viewer.hpp>

int main(int argc, char** argv) {
  const std::string config_ros_path = ament_index_cpp::get_package_share_directory("glim_ros") + "/config/glim_ros.json";
  std::cout << "config_ros_path:" << config_ros_path << std::endl;
  glim::Config config_ros(config_ros_path);

  const std::string config_path = ament_index_cpp::get_package_share_directory("glim") + config_ros.param<std::string>("glim_ros", "config_path", "/config");
  std::cout << "config_path:" << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);

  glim::OfflineViewer viewer;
  viewer.wait();
}