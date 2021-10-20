#include <any>
#include <deque>
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
#include <glim/frontend/async_odometry_estimation.hpp>
#include <glim/backend/async_sub_mapping.hpp>
#include <glim/backend/async_global_mapping.hpp>
#include <glim/frontend/odometry_estimation.hpp>
#include <glim/backend/sub_mapping.hpp>
#include <glim/backend/global_mapping.hpp>
#include <glim/viewer/interactive_viewer.hpp>

#include <glim_ros/raw_points.hpp>

class GlimROSNode : public rclcpp::Node {
public:
  GlimROSNode() : Node("glim_ros") {
    auto global_config = glim::GlobalConfig::instance();
    global_config->override_param<std::string>("global", "config_path", "/home/koide/ros2_ws/src/glim/config");

    latest_imu_stamp = 0.0;

    preprocessor.reset(new glim::CloudPreprocessor);

    odometry_estimation.reset(new glim::AsyncOdometryEstimation(std::shared_ptr<glim::OdometryEstimation>(new glim::OdometryEstimation)));
    sub_mapping.reset(new glim::AsyncSubMapping(std::shared_ptr<glim::SubMapping>(new glim::SubMapping)));
    global_mapping.reset(new glim::AsyncGlobalMapping(std::shared_ptr<glim::GlobalMapping>(new glim::GlobalMapping)));

    interactive_viewer.reset(new glim::InteractiveViewer);

    // const std::string points_topic = "/camera/depth/color/points";
    // const std::string imu_topic = "/camera/imu";
    const std::string imu_topic = "/livox/imu";
    const std::string points_topic = "/livox/lidar";
    const std::string image_topic = "/image";

    using std::placeholders::_1;
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 100, std::bind(&GlimROSNode::imu_callback, this, _1));
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(image_topic, 10, std::bind(&GlimROSNode::image_callback, this, _1));
    // points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, 10, std::bind(&GlimROSNode::points_callback, this, _1));
    points_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>(points_topic, 10, std::bind(&GlimROSNode::livox_points_callback, this, _1));

    timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&GlimROSNode::timer_callback, this));
  }

  ~GlimROSNode() {}

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    const double imu_acc_scale_factor = 9.80665;

    const double imu_stamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    const Eigen::Vector3d linear_acc = imu_acc_scale_factor * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3d angular_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    odometry_estimation->insert_imu(imu_stamp, linear_acc, angular_vel);
    sub_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);
    global_mapping->insert_imu(imu_stamp, linear_acc, angular_vel);

    latest_imu_stamp = imu_stamp;
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

    frame_queue.push_back(frame);

    while (!frame_queue.empty() && frame_queue.front()->stamp + 0.1 < latest_imu_stamp) {
      const auto front = frame_queue.front();
      auto preprocessed = preprocessor->preprocess(front->stamp, front->times, front->points);
      odometry_estimation->insert_frame(preprocessed);
      frame_queue.pop_front();
    }
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

    glim_ros::RawPoints::Ptr frame(new glim_ros::RawPoints);
    frame->stamp = stamp;
    frame->times = times;
    frame->points = points;

    frame_queue.push_back(frame);

    while (!frame_queue.empty() && frame_queue.front()->stamp + 0.1 < latest_imu_stamp) {
      const auto front = frame_queue.front();
      auto preprocessed = preprocessor->preprocess(front->stamp, front->times, front->points);
      odometry_estimation->insert_frame(preprocessed);
      frame_queue.pop_front();
    }
  }

  void timer_callback() {
    std::vector<glim::EstimationFrame::ConstPtr> estimation_frames;
    std::vector<glim::EstimationFrame::ConstPtr> marginalized_frames;
    odometry_estimation->get_results(estimation_frames, marginalized_frames);

    for(const auto& frame : marginalized_frames) {
      sub_mapping->insert_frame(frame);
    }

    auto submaps = sub_mapping->get_results();
    
    for(const auto& submap: submaps) {
      global_mapping->insert_submap(submap);
    }
  }

private:
  std::any timer;
  std::any imu_sub;
  std::any image_sub;
  std::any points_sub;

  double latest_imu_stamp;
  std::deque<glim_ros::RawPoints::ConstPtr> frame_queue;

  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;


  std::unique_ptr<glim::InteractiveViewer> interactive_viewer;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlimROSNode>());
  rclcpp::shutdown();
  return 0;
}