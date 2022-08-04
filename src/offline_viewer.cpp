#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/viewer/offline_viewer.hpp>

int main(int argc, char** argv) {
  std::string config_path = "config";
  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  std::cout << "config_path: " << config_path << std::endl;
  glim::GlobalConfig::instance(config_path);

  glim::OfflineViewer viewer;
  viewer.wait();
}