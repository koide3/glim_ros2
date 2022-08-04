#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));
  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "");

  // Create callbacks
  using std::placeholders::_1;
  const auto imu_callback = glim->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1000, [&](const sensor_msgs::msg::Imu::SharedPtr msg) { glim->imu_callback(msg); });
  const auto points_callback =
    glim->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, 30, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { glim->points_callback(msg); });

  for (const auto& sub : glim->extension_subscriptions()) {
    std::cout << "subscribe to " << sub->topic << std::endl;
    sub->create_subscriber(*glim);
  }

  rclcpp::spin(glim);
  rclcpp::shutdown();

  glim->wait();
  glim->save("/tmp/dump");

  return 0;
}