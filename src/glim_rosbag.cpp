#include <glob.h>
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>
#include <boost/format.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/glim_ros.hpp>

class SpeedCounter {
public:
  SpeedCounter() : last_sim_time(0.0), last_real_time(std::chrono::high_resolution_clock::now()) {}

  void update(const double& stamp) {
    const auto now = std::chrono::high_resolution_clock::now();
    if (now - last_real_time < std::chrono::seconds(5)) {
      return;
    }

    if (last_sim_time > 0.0) {
      const auto real = now - last_real_time;
      const auto sim = stamp - last_sim_time;
      const double playback_speed = sim / (std::chrono::duration_cast<std::chrono::nanoseconds>(real).count() / 1e9);
      spdlog::info("playback speed: {:.3f}x", playback_speed);
    }

    last_sim_time = stamp;
    last_real_time = now;
  }

private:
  double last_sim_time;
  std::chrono::high_resolution_clock::time_point last_real_time;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: glim_rosbag input_rosbag_path" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto glim = std::make_shared<glim::GlimROS>(options);

  // List topics
  glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));

  const std::string imu_topic = config_ros.param<std::string>("glim_ros", "imu_topic", "/imu");
  const std::string points_topic = config_ros.param<std::string>("glim_ros", "points_topic", "/points");
  const std::string image_topic = config_ros.param<std::string>("glim_ros", "image_topic", "/image");
  std::vector<std::string> topics = {imu_topic, points_topic, image_topic};

  rosbag2_storage::StorageFilter filter;
  spdlog::info("topics:");
  for (const auto& topic : topics) {
    spdlog::info("- {}", topic);
    filter.topics.push_back(topic);
  }

  //
  std::unordered_map<std::string, std::vector<glim::GenericTopicSubscription::Ptr>> subscription_map;
  for (const auto& sub : glim->extension_subscriptions()) {
    spdlog::info("- {} (ext)", sub->topic);
    filter.topics.push_back(sub->topic);
    subscription_map[sub->topic].push_back(sub);
  }

  // List input rosbag filenames
  std::vector<std::string> bag_filenames;

  for (int i = 1; i < argc; i++) {
    std::vector<std::string> filenames;
    glob_t globbuf;
    int ret = glob(argv[i], 0, nullptr, &globbuf);
    for (int i = 0; i < globbuf.gl_pathc; i++) {
      filenames.push_back(globbuf.gl_pathv[i]);
    }
    globfree(&globbuf);

    bag_filenames.insert(bag_filenames.end(), filenames.begin(), filenames.end());
  }
  std::sort(bag_filenames.begin(), bag_filenames.end());

  spdlog::info("bag_filenames:");
  for (const auto& bag_filename : bag_filenames) {
    spdlog::info("- {}", bag_filename);
  }

  const double playback_speed = config_ros.param<double>("glim_ros", "playback_speed", 100.0);
  const auto real_t0 = std::chrono::high_resolution_clock::now();
  rcutils_time_point_value_t bag_t0 = 0;
  SpeedCounter speed_counter;

  // Bag read function
  const auto read_bag = [&](const std::string& bag_filename) {
    spdlog::info("opening {}", bag_filename);
    rosbag2_cpp::Reader reader;
    reader.open(bag_filename);
    reader.set_filter(filter);

    const auto topics_and_types = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_type_map;
    for (const auto& topic : topics_and_types) {
      topic_type_map[topic.name] = topic.type;
    }

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> points_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serialization;

    while (reader.has_next()) {
      if (!rclcpp::ok()) {
        return false;
      }

      const auto msg = reader.read_next();
      const std::string topic_type = topic_type_map[msg->topic_name];
      const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

      if (bag_t0 == 0) {
        bag_t0 = msg->time_stamp;
      }

      const auto bag_elapsed = std::chrono::nanoseconds(msg->time_stamp - bag_t0);
      while (playback_speed > 0.0 && (std::chrono::high_resolution_clock::now() - real_t0) * playback_speed < bag_elapsed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (msg->topic_name == imu_topic) {
        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());
        glim->imu_callback(imu_msg);
      } else if (msg->topic_name == points_topic) {
        auto points_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        points_serialization.deserialize_message(&serialized_msg, points_msg.get());
        glim->points_callback(points_msg);
      } else if (msg->topic_name == image_topic && topic_type == "sensor_msgs/msg/Image") {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serialization.deserialize_message(&serialized_msg, image_msg.get());
        glim->image_callback(image_msg);
      } else if (msg->topic_name == image_topic && topic_type == "sensor_msgs/msg/CompressedImage") {
        auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_image_serialization.deserialize_message(&serialized_msg, compressed_image_msg.get());
      }

      auto found = subscription_map.find(msg->topic_name);
      if (found != subscription_map.end()) {
        for (const auto& sub : found->second) {
          sub->insert_message_instance(serialized_msg);
        }
      }

      glim->timer_callback();
      speed_counter.update(msg->time_stamp / 1e9);

      const auto t0 = std::chrono::high_resolution_clock::now();
      while (glim->needs_wait()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (std::chrono::high_resolution_clock::now() - t0 > std::chrono::seconds(1)) {
          spdlog::warn("throttling timeout (an extension module may be hanged)");
          break;
        }
      }
    }

    return true;
  };

  // Read all rosbags
  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename)) {
      break;
    }
  }

  bool auto_quit = false;
  glim->declare_parameter<bool>("auto_quit", auto_quit);
  glim->get_parameter<bool>("auto_quit", auto_quit);

  if (!auto_quit) {
    rclcpp::spin(glim);
  }

  glim->wait(auto_quit);
  glim->save("/tmp/dump");

  return 0;
}