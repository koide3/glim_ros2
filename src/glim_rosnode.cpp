#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);
  exec.add_node(glim);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}