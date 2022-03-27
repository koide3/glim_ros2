#pragma once

#include <any>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/frontend/estimation_frame.hpp>
#include <glim/backend/sub_map.hpp>

namespace glim {

class TrajectoryManager;

class RvizViewer {
public:
  RvizViewer(rclcpp::Node& node);
  ~RvizViewer();

private:
  void set_callbacks();
  void frontend_new_frame(const EstimationFrame::ConstPtr& new_frame);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void invoke(const std::function<void()>& task);

  void spin_once();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string odom_frame_id;
  std::string world_frame_id;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_ext::Frame::ConstPtr> submaps;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;
};
}  // namespace glim