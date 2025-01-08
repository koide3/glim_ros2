#include <iostream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/viewer/offline_viewer.hpp>

int main(int argc, char** argv) {
  // Setup logger
  auto logger = spdlog::stdout_color_mt("glim");
  logger->sinks().push_back(glim::get_ringbuffer_sink());
  spdlog::set_default_logger(logger);

  std::string config_path = "config";
  if (config_path[0] != '/') {
    // config_path is relative to the glim directory
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/" + config_path;
  }

  spdlog::info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);

  std::string init_map_path;
  if (argc >= 2) {
    init_map_path = argv[1];
    spdlog::info("map_path={}", init_map_path);
  }

  glim::OfflineViewer viewer(init_map_path);
  viewer.wait();
}