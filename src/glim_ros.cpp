#include <any>
#include <thread>
#include <iostream>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_interfaces/msg/custom_msg.hpp>

#include <glim/util/config.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/frontend/odometry_estimation.hpp>
#include <glim/viewer/interactive_viewer.hpp>

#include <glim_ros/raw_points.hpp>

class GlimROSNode : public rclcpp::Node {
public:
  GlimROSNode() : Node("glim_ros") {
    auto global_config = glim::GlobalConfig::instance();
    global_config->override_param<std::string>("global", "config_path", "/home/koide/ros2_ws/src/glim/config");

    preprocessor.reset(new glim::CloudPreprocessor);
    odometry_estimation.reset(new glim::OdometryEstimation);
    interactive_viewer.reset(new glim::InteractiveViewer);

    using std::placeholders::_1;
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu", 100, std::bind(&GlimROSNode::imu_callback, this, _1));
    image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, std::bind(&GlimROSNode::image_callback, this, _1));
    // points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&GlimROSNode::points_callback, this, _1));
    points_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>("/livox/lidar", 10, std::bind(&GlimROSNode::livox_points_callback, this, _1));
  }

  ~GlimROSNode() {}

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    const Eigen::Vector3d linear_acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    odometry_estimation->insert_imu(imu_stamp, linear_acc * 9.80665, angular_vel);
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) { auto cv_image = cv_bridge::toCvCopy(msg, "bgr8"); }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    auto frame = glim_ros::RawPoints::extract(msg);
    if (frame == nullptr) {
      RCLCPP_WARN_STREAM(this->get_logger(), "failed to extract points from message");
      return;
    }

    const double scan_duration = 0.1;
    if(frame->times.empty()) {
      frame->times.resize(frame->points.size());
      for(int i=0; i<frame->points.size(); i++) {
        frame->times[i] = scan_duration * static_cast<double>(i) / frame->points.size();
      }
    }

    auto preprocessed = preprocessor->preprocess(frame->stamp, frame->times, frame->points);
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    auto estimated = odometry_estimation->insert_frame(preprocessed, marginalized_frames);
  }

  void livox_points_callback(const livox_interfaces::msg::CustomMsg::ConstSharedPtr msg) {
    const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    std::vector<double> times(msg->point_num);
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points(msg->point_num);

    for (int i = 0; i < msg->point_num; i++) {
      const auto& point = msg->points[i];
      times[i] = point.offset_time / 1e9;
      points[i] << point.x, point.y, point.z, 1.0;
    }

    auto preprocessed = preprocessor->preprocess(stamp, times, points);
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    auto estimated = odometry_estimation->insert_frame(preprocessed, marginalized_frames);
  }

private:
  std::any timer;
  std::any imu_sub;
  std::any image_sub;
  std::any points_sub;

  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::OdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::InteractiveViewer> interactive_viewer;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlimROSNode>());
  rclcpp::shutdown();
  return 0;
}