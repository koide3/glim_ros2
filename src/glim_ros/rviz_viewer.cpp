#include <glim_ros/rviz_viewer.hpp>

#include <mutex>
#include <rclcpp/clock.hpp>

#define GLIM_ROS2
#include <gtsam_ext/types/frame_cpu.hpp>
#include <glim/frontend/callbacks.hpp>
#include <glim/backend/callbacks.hpp>
#include <glim/util/trajectory_manager.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

RvizViewer::RvizViewer(rclcpp::Node& node) {
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  points_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("/glim_ros/points", 10);
  aligned_points_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("/glim_ros/aligned_points", 10);

  rmw_qos_profile_t map_qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};
  rclcpp::QoS map_qos(rclcpp::QoSInitialization(map_qos_profile.history, map_qos_profile.depth), map_qos_profile);
  map_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("/glim_ros/map", map_qos);

  odom_pub = node.create_publisher<nav_msgs::msg::Odometry>("/glim_ros/odom", 10);
  pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("/glim_ros/pose", 10);

  imu_frame_id = "imu";
  lidar_frame_id = "lidar";
  odom_frame_id = "odom";
  world_frame_id = "world";

  last_globalmap_pub_time = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();
  trajectory.reset(new TrajectoryManager);

  set_callbacks();

  kill_switch = false;
  thread = std::thread([this] {
    while (!kill_switch) {
      const auto expected = std::chrono::milliseconds(10);
      const auto t1 = std::chrono::high_resolution_clock::now();
      spin_once();
      const auto t2 = std::chrono::high_resolution_clock::now();

      if (t2 - t1 < expected) {
        std::this_thread::sleep_for(expected - (t2 - t1));
      }
    }
  });
}

RvizViewer::~RvizViewer() {
  kill_switch = true;
  thread.join();
}

void RvizViewer::set_callbacks() {
  using std::placeholders::_1;
  OdometryEstimationCallbacks::on_new_frame.add(std::bind(&RvizViewer::frontend_new_frame, this, _1));
  GlobalMappingCallbacks::on_update_submaps.add(std::bind(&RvizViewer::globalmap_on_update_submaps, this, _1));
}

void RvizViewer::frontend_new_frame(const EstimationFrame::ConstPtr& new_frame) {
  const Eigen::Isometry3d T_odom_imu = new_frame->T_world_imu;
  const Eigen::Quaterniond quat_odom_imu(T_odom_imu.linear());

  const Eigen::Isometry3d T_lidar_imu = new_frame->T_lidar_imu;
  const Eigen::Quaterniond quat_lidar_imu(T_lidar_imu.linear());

  Eigen::Isometry3d T_world_odom;
  Eigen::Quaterniond quat_world_odom;

  Eigen::Isometry3d T_world_imu;
  Eigen::Quaterniond quat_world_imu;

  {
    // Transform the frontend frame to the global optimization-based world frame
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->add_odom(new_frame->stamp, new_frame->T_world_imu);
    T_world_odom = trajectory->get_T_world_odom();
    quat_world_odom = Eigen::Quaterniond(T_world_odom.linear());

    T_world_imu = trajectory->odom2world(T_odom_imu);
    quat_world_imu = Eigen::Quaterniond(T_world_imu.linear());
  }

  // Publish transforms
  // Odom -> IMU
  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = rclcpp::Time(new_frame->stamp);
  trans.header.frame_id = odom_frame_id;
  trans.child_frame_id = imu_frame_id;
  trans.transform.translation.x = T_odom_imu.translation().x();
  trans.transform.translation.y = T_odom_imu.translation().y();
  trans.transform.translation.z = T_odom_imu.translation().z();
  trans.transform.rotation.x = quat_odom_imu.x();
  trans.transform.rotation.y = quat_odom_imu.y();
  trans.transform.rotation.z = quat_odom_imu.z();
  trans.transform.rotation.w = quat_odom_imu.w();
  tf_broadcaster->sendTransform(trans);

  // IMU -> LiDAR
  trans.header.frame_id = imu_frame_id;
  trans.child_frame_id = lidar_frame_id;
  trans.transform.translation.x = T_lidar_imu.translation().x();
  trans.transform.translation.y = T_lidar_imu.translation().y();
  trans.transform.translation.z = T_lidar_imu.translation().z();
  trans.transform.rotation.x = quat_lidar_imu.x();
  trans.transform.rotation.y = quat_lidar_imu.y();
  trans.transform.rotation.z = quat_lidar_imu.z();
  trans.transform.rotation.w = quat_lidar_imu.w();
  tf_broadcaster->sendTransform(trans);

  // World -> Odom
  trans.header.frame_id = world_frame_id;
  trans.child_frame_id = odom_frame_id;
  trans.transform.translation.x = T_world_odom.translation().x();
  trans.transform.translation.y = T_world_odom.translation().y();
  trans.transform.translation.z = T_world_odom.translation().z();
  trans.transform.rotation.x = quat_world_odom.x();
  trans.transform.rotation.y = quat_world_odom.y();
  trans.transform.rotation.z = quat_world_odom.z();
  trans.transform.rotation.w = quat_world_odom.w();
  tf_broadcaster->sendTransform(trans);

  if (odom_pub->get_subscription_count()) {
    // Publish sensor pose (without loop closure)
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(new_frame->stamp);
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = imu_frame_id;
    odom.pose.pose.position.x = T_odom_imu.translation().x();
    odom.pose.pose.position.y = T_odom_imu.translation().y();
    odom.pose.pose.position.z = T_odom_imu.translation().z();
    odom.pose.pose.orientation.x = quat_odom_imu.x();
    odom.pose.pose.orientation.y = quat_odom_imu.y();
    odom.pose.pose.orientation.z = quat_odom_imu.z();
    odom.pose.pose.orientation.w = quat_odom_imu.w();
    odom_pub->publish(odom);
  }

  if (pose_pub->get_subscription_count()) {
    // Publish sensor pose (with loop closure)
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Time(new_frame->stamp);
    pose.header.frame_id = world_frame_id;
    pose.pose.position.x = T_world_imu.translation().x();
    pose.pose.position.y = T_world_imu.translation().y();
    pose.pose.position.z = T_world_imu.translation().z();
    pose.pose.orientation.x = quat_world_imu.x();
    pose.pose.orientation.y = quat_world_imu.y();
    pose.pose.orientation.z = quat_world_imu.z();
    pose.pose.orientation.w = quat_world_imu.w();
    pose_pub->publish(pose);
  }

  if (points_pub->get_subscription_count()) {
    // Publish points in their own coordinate frame
    std::string frame_id;
    switch (new_frame->frame_id) {
      case FrameID::LIDAR:
        frame_id = lidar_frame_id;
        break;
      case FrameID::IMU:
        frame_id = imu_frame_id;
        break;
      case FrameID::WORLD:
        frame_id = world_frame_id;
        break;
    }

    auto points = frame_to_pointcloud2(frame_id, new_frame->stamp, *new_frame->frame);
    points_pub->publish(*points);
  }

  if (aligned_points_pub->get_subscription_count()) {
    // Publish points aligned to the world frame to avoid some visualization issues in Rviz2
    std::vector<Eigen::Vector4d> transformed(new_frame->frame->size());
    for (int i = 0; i < new_frame->frame->size(); i++) {
      transformed[i] = new_frame->T_world_sensor() * new_frame->frame->points[i];
    }

    gtsam_ext::Frame frame;
    frame.num_points = new_frame->frame->size();
    frame.points = transformed.data();
    frame.times = new_frame->frame->times;
    frame.intensities = new_frame->frame->intensities;

    auto points = frame_to_pointcloud2(world_frame_id, new_frame->stamp, frame);
    aligned_points_pub->publish(*points);
  }
}

void RvizViewer::globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  const SubMap::ConstPtr latest_submap = submaps.back();

  const double stamp_endpoint_R = latest_submap->odom_frames.back()->stamp;
  const Eigen::Isometry3d T_world_endpoint_R = latest_submap->T_world_origin * latest_submap->T_origin_endpoint_R;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    trajectory->update_anchor(stamp_endpoint_R, T_world_endpoint_R);
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses(submaps.size());
  for (int i = 0; i < submaps.size(); i++) {
    submap_poses[i] = submaps[i]->T_world_origin;
  }

  // Invoke a submap concatenation task in the RvizViewer thread
  invoke([this, latest_submap, submap_poses] {
    this->submaps.push_back(latest_submap->frame);

    if (!map_pub->get_subscription_count()) {
      return;
    }

    // Publish global map every 30 seconds
    const rclcpp::Time now = rclcpp::Clock(rcl_clock_type_t::RCL_ROS_TIME).now();
    if (now - last_globalmap_pub_time < std::chrono::seconds(30)) {
      return;
    }
    last_globalmap_pub_time = now;

    int total_num_points = 0;
    for (const auto& submap : this->submaps) {
      total_num_points += submap->size();
    }

    // Concatenate all the submap points
    gtsam_ext::FrameCPU::Ptr merged(new gtsam_ext::FrameCPU);
    merged->num_points = total_num_points;
    merged->points_storage.resize(total_num_points);
    merged->points = merged->points_storage.data();

    int begin = 0;
    for (int i = 0; i < this->submaps.size(); i++) {
      const auto& submap = this->submaps[i];
      std::transform(submap->points, submap->points + submap->size(), merged->points + begin, [&](const Eigen::Vector4d& p) { return submap_poses[i] * p; });
      begin += submap->size();
    }

    auto points_msg = frame_to_pointcloud2(world_frame_id, now.seconds(), *merged);
    map_pub->publish(*points_msg);
  });
}

void RvizViewer::invoke(const std::function<void()>& task) {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  invoke_queue.push_back(task);
}

void RvizViewer::spin_once() {
  std::lock_guard<std::mutex> lock(invoke_queue_mutex);
  for (const auto& task : invoke_queue) {
    task();
  }
  invoke_queue.clear();
}

}  // namespace glim