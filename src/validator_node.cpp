#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define GLIM_ROS2
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/util/data_validator.hpp>

namespace glim {

class ValidatorNode : public rclcpp::Node {
public:
  ValidatorNode(rclcpp::NodeOptions& options) : rclcpp::Node("validator_node", options) {
    bool debug = false;
    this->declare_parameter("debug", debug);
    this->get_parameter("debug", debug);

    validator.reset(new DataValidator(debug));

    using std::placeholders::_1;
    const std::string imu_topic = "imu";
    const std::string points_topic = "points";

    auto imu_qos = rclcpp::SensorDataQoS();
    imu_qos.get_rmw_qos_profile().depth = 1000;
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, imu_qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
      const auto& a = msg->linear_acceleration;
      const auto& w = msg->angular_velocity;
      validator->imu_callback(stamp, {a.x, a.y, a.z}, {w.x, w.y, w.z});
    });
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
      auto raw_points = extract_raw_points(msg);
      validator->points_callback(stamp, raw_points);
    });

    timer = this->create_wall_timer(std::chrono::seconds(1), [this]() { validator->timer_callback(); });
  }

private:
  std::unique_ptr<DataValidator> validator;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
};

}  // namespace glim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  rclcpp::spin(std::make_shared<glim::ValidatorNode>(options));
  rclcpp::shutdown();

  return 0;
}